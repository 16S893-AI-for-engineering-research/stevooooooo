// Keep a flock alive until the parent closes stdin. No background polling/model calls.
process.stdin.resume();
process.stdout.write('locked\n');
process.stdin.on('end', () => process.exit(0));
