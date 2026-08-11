const fs = require('node:fs');
const path = require('node:path');
const { spawnSync } = require('node:child_process');

const desktopRoot = path.resolve(__dirname, '..');
const appPath = path.join(
  desktopRoot,
  'out',
  'F1 Race Manager-darwin-arm64',
  'F1 Race Manager.app',
);

if (process.platform !== 'darwin') {
  throw new Error('local package signing is supported only on macOS');
}
if (!fs.existsSync(appPath)) {
  throw new Error(`packaged app is missing: ${appPath}`);
}

for (const args of [
  ['--force', '--deep', '--sign', '-', appPath],
  ['--verify', '--deep', '--strict', '--verbose=2', appPath],
]) {
  const result = spawnSync('/usr/bin/codesign', args, { stdio: 'inherit' });
  if (result.status !== 0) {
    throw new Error(`codesign failed with exit status ${result.status}`);
  }
}

console.log(`Applied and verified local ad-hoc signature: ${appPath}`);
