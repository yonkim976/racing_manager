const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('desktopDiagnostics', Object.freeze({
  recordRendererSnapshot: (snapshot) => ipcRenderer.invoke('desktop:renderer-snapshot', snapshot),
  recordCheckpoint: (name, details) => ipcRenderer.invoke('desktop:checkpoint', { name, details }),
  getLifecycleState: () => ipcRenderer.invoke('desktop:lifecycle-state'),
  getLogLocation: () => ipcRenderer.invoke('desktop:log-location'),
}));
