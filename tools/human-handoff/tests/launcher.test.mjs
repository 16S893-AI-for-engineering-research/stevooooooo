import test from 'node:test';
import assert from 'node:assert/strict';
import { mkdtemp, mkdir, writeFile, readFile, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { openGuide, resolveProviderExtension, harvestShellCredentials } from '../launcher.mjs';
import { readJSON, saveJSON } from '../core.mjs';

async function fixture(t) {
  const root = await mkdtemp(join(tmpdir(), 'handoff-launch-test-'));
  const bin = join(root, 'bin'), home = join(root, 'state'), calls = join(root, 'calls.jsonl');
  await mkdir(bin);
  await writeFile(join(bin, 'wt.exe'), `#!${process.execPath}\nrequire('node:fs').appendFileSync(${JSON.stringify(calls)}, JSON.stringify(process.argv.slice(2))+'\\n');process.exit(Number(process.env.MOCK_WT_EXIT || 0));\n`, { mode: 0o700 });
  const path = process.env.PATH;
  process.env.PATH = bin + ':' + path;
  t.after(async () => { process.env.PATH = path; delete process.env.MOCK_WT_EXIT; await rm(root, { recursive: true, force: true }); });
  return { root, home, calls };
}

test('concurrent accepted actions request only one pending tab launch', async t => {
  const f = await fixture(t);
  const config = { window: '0', distro: 'Ubuntu' };
  const responses = await Promise.all([openGuide(f.home, f.root, config), openGuide(f.home, f.root, config)]);
  assert.equal((await readFile(f.calls, 'utf8')).trim().split('\n').length, 1);
  assert.ok(responses.some(r => r.includes('pending')));
  assert.equal((await readJSON(join(f.home, 'launch.json'))).status, 'pending');
});

test('failed launches stay pending until explicit retry; stopped guide can reopen', async t => {
  const f = await fixture(t);
  const config = { window: '0', distro: 'Ubuntu' };
  process.env.MOCK_WT_EXIT = '1';
  await assert.rejects(openGuide(f.home, f.root, config), /could not be confirmed/);
  delete process.env.MOCK_WT_EXIT;
  assert.match(await openGuide(f.home, f.root, config), /pending/);
  assert.match(await openGuide(f.home, f.root, config, { retry: true }), /Requested/);
  const launch = await readJSON(join(f.home, 'launch.json'));
  await saveJSON(join(f.home, 'launch.json'), { ...launch, status: 'stopped' });
  assert.match(await openGuide(f.home, f.root, config), /Requested/);
  assert.equal((await readFile(f.calls, 'utf8')).trim().split('\n').length, 3);
});

test('provider extension is resolved so the guide can register an extension-contributed model', async t => {
  const root = await mkdtemp(join(tmpdir(), 'handoff-provider-test-'));
  t.after(() => rm(root, { recursive: true, force: true }));
  const pkg = join(root, 'some-pi-extension');
  await mkdir(join(pkg, 'dist'), { recursive: true });
  await writeFile(join(pkg, 'dist', 'index.js'), 'export const provider = "MIT-parley";\n');
  // Unrelated package must not be matched.
  const other = join(root, 'other-pi-extension');
  await mkdir(join(other, 'dist'), { recursive: true });
  await writeFile(join(other, 'dist', 'index.js'), 'export const provider = "something-else";\n');

  assert.equal(await resolveProviderExtension('MIT-parley/bedrock/claude-opus-5', root), pkg);
  assert.equal(await resolveProviderExtension('unknown-provider/model', root), null);
  assert.equal(await resolveProviderExtension(undefined, root), null);
  assert.equal(await resolveProviderExtension('MIT-parley/x', join(root, 'missing')), null);
});

test('launch records the provider extension so a restart does not lose model resolution', async t => {
  const f = await fixture(t);
  const packages = join(f.root, 'packages');
  const pkg = join(packages, 'mit-parley-pi-extension');
  await mkdir(join(pkg, 'dist'), { recursive: true });
  await writeFile(join(pkg, 'dist', 'index.js'), 'export const provider = "MIT-parley";\n');
  const launch = {
    model: 'MIT-parley/bedrock/claude-opus-5',
    providerExtension: await resolveProviderExtension('MIT-parley/bedrock/claude-opus-5', packages),
  };
  assert.equal(launch.providerExtension, pkg);

  // The runner must emit -e for that extension and keep --model.
  const args = ['--no-extensions'];
  if (launch.providerExtension) args.push('-e', launch.providerExtension);
  if (launch.model && (launch.providerExtension || !launch.model.includes('/'))) args.push('--model', launch.model);
  assert.deepEqual(args, ['--no-extensions', '-e', pkg, '--model', 'MIT-parley/bedrock/claude-opus-5']);

  // With no resolvable provider, --model must be dropped rather than passed and failing.
  const bare = ['--no-extensions'];
  const unresolved = { model: 'MIT-parley/bedrock/claude-opus-5', providerExtension: null };
  if (unresolved.model && (unresolved.providerExtension || !unresolved.model.includes('/'))) bare.push('--model', unresolved.model);
  assert.deepEqual(bare, ['--no-extensions']);
});

test('shell credentials are harvested because wsl.exe --exec skips shell init', async t => {
  const root = await mkdtemp(join(tmpdir(), 'handoff-cred-test-'));
  t.after(() => rm(root, { recursive: true, force: true }));
  // Stand-in for an interactive login shell that exports a credential.
  const shell = join(root, 'fakeshell');
  await writeFile(shell, `#!${process.execPath}\nprocess.stdout.write([
    'PARLEY_API_KEY=sk-test-value',
    'MIT_PARLEY_BASE_URL=https://example.invalid/v1',
    'HOME=/home/someone',
    'EDITOR=vim',
    'EMPTY_API_KEY=',
  ].join('\\0'));\n`, { mode: 0o700 });

  const got = await harvestShellCredentials({}, shell);
  assert.equal(got.PARLEY_API_KEY, 'sk-test-value');
  assert.equal(got.MIT_PARLEY_BASE_URL, 'https://example.invalid/v1');
  // Only credential-shaped names, never arbitrary environment.
  assert.equal(got.HOME, undefined);
  assert.equal(got.EDITOR, undefined);
  // Blank values are not treated as credentials.
  assert.equal(got.EMPTY_API_KEY, undefined);

  // An already-set credential must win; the guide must not be silently re-keyed.
  const existing = await harvestShellCredentials({ PARLEY_API_KEY: 'from-parent' }, shell);
  assert.equal(existing.PARLEY_API_KEY, undefined);

  // A broken shell must not prevent the guide from starting.
  assert.deepEqual(await harvestShellCredentials({}, join(root, 'does-not-exist')), {});
});

test('harvested credentials are never written to launch.json', async t => {
  const f = await fixture(t);
  process.env.HANDOFF_TEST_FAKE_API_KEY = 'sk-should-not-persist';
  t.after(() => { delete process.env.HANDOFF_TEST_FAKE_API_KEY; });
  await openGuide(f.home, f.root, { window: '0', distro: 'Ubuntu' });
  const raw = await readFile(join(f.home, 'launch.json'), 'utf8');
  assert.ok(!raw.includes('sk-should-not-persist'), 'launch.json must not contain credential values');
  assert.ok(!/API_KEY/.test(raw), 'launch.json must not contain credential variable names');
});
