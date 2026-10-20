// Invoked inside WSL by `flock -n guide.lock node guide-runner.mjs HOME`.
// Replace this process with Pi, rather than leaving an orphanable wrapper/child pair.
// The outer flock waits on the same PID for the whole interactive Pi lifetime.
import { join } from 'node:path';
import { readJSON, saveJSON, now } from './core.mjs';
const home = process.argv[2];
if (!home) throw new Error('Missing private handoff state directory');
if (typeof process.execve !== 'function') throw new Error('Walkthrough runner requires Node >=22.15 with process.execve');
const config = await readJSON(join(home, 'launch.json'), null);
if (!config) throw new Error('No launch request');
const args = [
  '--no-extensions', '-e', config.extension, '--no-skills', '--no-prompt-templates', '--no-context-files',
  '--tools', 'read,human_queue,human_checkpoint',
  '--append-system-prompt', config.profile, '--session-dir', join(home, 'sessions'),
  '--continue', '--name', 'Walkthrough',
];
if (config.model) args.push('--model', config.model);
const env = { ...process.env, PATH: config.path, PI_HUMAN_GUIDE: '1', PI_HUMAN_STATE_HOME: home };
for (const key of ['PI_SESSION_ID', 'PI_SESSION_FILE', 'PI_PROVIDER', 'PI_MODEL', 'PI_REASONING_LEVEL']) delete env[key];
process.chdir(config.repo);
await saveJSON(join(home, 'launch.json'), { ...config, status: 'running', pid: process.pid, startedAt: now() });
// guide.lock, not the last-written launch status, is authoritative after shutdown/crash.
process.execve(config.pi, [config.pi, ...args], env);
