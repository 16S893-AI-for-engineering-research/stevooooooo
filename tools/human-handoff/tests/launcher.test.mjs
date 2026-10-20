import test from 'node:test';
import assert from 'node:assert/strict';
import { mkdtemp, mkdir, writeFile, readFile, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { openGuide } from '../launcher.mjs';
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
