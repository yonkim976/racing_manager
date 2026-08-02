const fs = require('node:fs');
const path = require('node:path');

const desktopRoot = path.resolve(__dirname, '..');
const projectRoot = path.resolve(desktopRoot, '..');
const frontendSource = path.join(projectRoot, 'frontend', 'dist');
const backendSource = path.join(projectRoot, 'backend');
const pythonSource = path.join(backendSource, '.venv');
const resourcesRoot = path.join(desktopRoot, 'resources');

function copyTree(source, destination, filter) {
  fs.cpSync(source, destination, { recursive: true, dereference: true, filter });
}

if (!fs.existsSync(path.join(frontendSource, 'index.html'))) {
  throw new Error('frontend/dist/index.html is missing; build the frontend first');
}
if (!fs.existsSync(path.join(pythonSource, 'bin', 'python'))) {
  throw new Error('backend/.venv/bin/python is required for the one-directory package');
}

for (const directory of ['frontend-dist', 'backend', 'python']) {
  fs.rmSync(path.join(resourcesRoot, directory), { recursive: true, force: true });
}
fs.mkdirSync(resourcesRoot, { recursive: true });
copyTree(frontendSource, path.join(resourcesRoot, 'frontend-dist'));
const ignoredBackendDirectories = new Set([
  '.git',
  '.pytest_cache',
  '.mypy_cache',
  '.ruff_cache',
]);
copyTree(backendSource, path.join(resourcesRoot, 'backend'), (source) => {
  const relative = path.relative(backendSource, source);
  const parts = relative.split(path.sep);
  return relative !== '.venv'
    && !parts.includes('__pycache__')
    && !parts.includes('tests')
    && !parts.some((part) => ignoredBackendDirectories.has(part));
});
copyTree(pythonSource, path.join(resourcesRoot, 'python'));
console.log(`Prepared packaged frontend, backend and Python runtime under ${resourcesRoot}`);
