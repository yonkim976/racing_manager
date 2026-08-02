const fs = require('node:fs');
const path = require('node:path');
const { execFileSync } = require('node:child_process');

const desktopRoot = path.resolve(__dirname, '..');
const electronApp = path.join(desktopRoot, 'node_modules', 'electron', 'dist', 'Electron.app');
const cacheRoot = path.join(desktopRoot, '.electron-cache');
const zipPath = path.join(cacheRoot, 'electron-v37.2.6-darwin-arm64.zip');

if (process.platform !== 'darwin') throw new Error('The macOS package target requires a macOS host');
if (!fs.existsSync(electronApp)) throw new Error(`Electron.app not found at ${electronApp}`);
fs.mkdirSync(cacheRoot, { recursive: true });
fs.rmSync(zipPath, { force: true });
execFileSync('/usr/bin/ditto', [
  '-c', '-k', '--sequesterRsrc', '--keepParent', electronApp, zipPath,
], { stdio: 'inherit' });
console.log(`Prepared local Electron archive at ${zipPath}`);
