import { access, mkdir, readdir, readFile } from 'node:fs/promises';
import { constants } from 'node:fs';
import { execFile } from 'node:child_process';
import { promisify } from 'node:util';
import { join, dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { homedir } from 'node:os';
import { locked, readJSON, saveJSON, now } from './core.mjs';
const exec = promisify(execFile);
const HERE = dirname(fileURLToPath(import.meta.url));

async function executable(name) {
  for (const dir of (process.env.PATH || '').split(':')) {
    const candidate = join(dir, name);
    try { await access(candidate, constants.X_OK); return candidate; } catch { /* next */ }
  }
  throw new Error(`${name} is not available on PATH`);
}

// The guide runs with --no-extensions, so a provider contributed by an extension
// (for example MIT-parley) is not registered there and --model cannot resolve.
// Find the installed package that declares the provider so the runner can re-add
// just that one extension. Returns null when the provider is built in or unknown,
// in which case the runner drops --model rather than failing to start.
export async function resolveProviderExtension(model, packagesDir = join(homedir(), '.pi', 'agent', 'npm', 'node_modules')) {
  const provider = String(model || '').split('/')[0];
  if (!provider || !/^[\w.-]+$/.test(provider)) return null;
  let entries;
  try { entries = await readdir(packagesDir, { withFileTypes: true }); } catch { return null; }
  for (const entry of entries) {
    if (!entry.isDirectory() || !entry.name.includes('pi-extension')) continue;
    const dir = join(packagesDir, entry.name);
    for (const rel of ['dist/index.js', 'src/index.ts', 'index.js', 'index.mjs']) {
      try {
        if ((await readFile(join(dir, rel), 'utf8')).includes(provider)) return dir;
      } catch { /* next candidate */ }
    }
  }
  return null;
}
// The guide is launched through `wsl.exe --exec`, which does not run shell
// initialization, so provider credentials exported from ~/.bashrc or ~/.profile
// are absent and pi reports "No API key found for the selected model". Harvest
// only the missing credential-shaped variables from an interactive login shell,
// in-memory, so nothing is copied into launch.json on disk.
const CREDENTIAL_VAR = /(?:_API_KEY|_BASE_URL|_AUTH_TOKEN|_OAUTH_TOKEN)$/;
export async function harvestShellCredentials(currentEnv = process.env, shell = '/bin/bash') {
  let stdout;
  try {
    ({ stdout } = await exec(shell, ['-ic', 'env -0'], { timeout: 10000, maxBuffer: 1 << 20 }));
  } catch (e) {
    // A missing or non-cooperative shell must not stop the guide from starting.
    if (!e.stdout) return {};
    stdout = e.stdout;
  }
  const found = {};
  for (const pair of String(stdout).split('\0')) {
    const eq = pair.indexOf('=');
    if (eq <= 0) continue;
    const name = pair.slice(0, eq);
    if (!CREDENTIAL_VAR.test(name)) continue;
    if (currentEnv[name]?.trim()) continue; // never override what is already set
    const value = pair.slice(eq + 1);
    if (value.trim()) found[name] = value;
  }
  return found;
}

export function terminalArgs(config, home, node = process.execPath, runner = join(HERE, 'guide-runner.mjs')) {
  // No shell string interpolation. Window 0 is WT's most recently used window, not necessarily the originating window.
  return ['-w', config.window, 'new-tab', '--title', 'Pi Walkthrough', 'wsl.exe', '-d', config.distro, '--exec', '/usr/bin/flock', '-n', join(home, 'guide.lock'), node, runner, home];
}
export async function guideRunning(home) {
  await mkdir(home, { recursive: true, mode: 0o700 });
  try { await exec('flock', ['-n', join(home, 'guide.lock'), 'true'], { timeout: 3000 }); return false; }
  catch (e) { if (e.code === 1) return true; throw e; }
}
export async function openGuide(home, repo, config, options = {}) {
  return locked(home, async () => {
    if (await guideRunning(home)) return 'The existing Walkthrough tab owns the guide. Task queued; no new tab opened.';
    const pending = await readJSON(join(home, 'launch.json'), null);
    if (pending?.status === 'pending' && !options.retry) return 'Tab launch is pending. If it never appeared, use /human retry to explicitly retry; no duplicate tab was opened.';
    if (!config.distro) throw new Error('No WSL distribution detected. Configure with /human setup.');
    if (!/^[\w.-]+$/.test(config.distro) || !/^[\w.-]+$/.test(config.window)) throw new Error('Invalid WSL distribution or Windows Terminal window target');
    const launch = {
      repo: resolve(repo), distro: config.distro, window: config.window,
      pi: await executable('pi'), node: process.execPath, path: process.env.PATH,
      extension: resolve(HERE, '../../.pi/extensions/human-handoff/index.ts'),
      profile: join(HERE, 'walkthrough.md'),
      model: config.model || null, providerExtension: await resolveProviderExtension(config.model), status: 'pending', requestedAt: now(),
    };
    await saveJSON(join(home, 'launch.json'), launch);
    try {
      await exec(await executable('wt.exe'), terminalArgs(config, home), { timeout: 10000, maxBuffer: 10000 });
    } catch (e) {
      // A timeout may have launched WT successfully. Keep pending to prevent blind duplicate launches.
      await saveJSON(join(home, 'launch.json'), { ...launch, error: String(e.message).slice(0, 1000) });
      throw new Error(`Task remains accepted, but tab launch could not be confirmed: ${e.message}. Use /human retry after checking Windows Terminal.`);
    }
    return 'Requested a Walkthrough tab. The task stays queued until the guide starts; /human status shows readiness.';
  });
}
