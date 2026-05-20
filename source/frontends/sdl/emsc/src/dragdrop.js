Module.init_dragdrop = function () {
  console.log('[DND] Drag & drop initialized');

  function prevent(e) {
    e.preventDefault();
    e.stopPropagation();
    return false;
  }

  // Convert JS string to UTF8 pointer for WASM
  function cstr(str) {
    const len = lengthBytesUTF8(str) + 1;
    const ptr = _malloc(len);
    stringToUTF8(str, ptr, len);
    return ptr;
  }

  const DROP_PATH = '/disks';

  // Ensure drop directory exists in MEMFS
  if (!FS.analyzePath(DROP_PATH).exists) {
    FS.mkdir(DROP_PATH);
    console.log('[DND] Created ' + DROP_PATH + ' directory in MEMFS');
  }

  // Attach global handlers
  document.addEventListener('dragenter', prevent, false);
  document.addEventListener('dragover', prevent, false);
  document.addEventListener('dragleave', prevent, false);

  document.addEventListener('drop', (e) => {
    prevent(e);

    const files = e.dataTransfer.files;
    console.log('[DND] drop event, files:', files.length);

    for (let i = 0; i < files.length; ++i) {
      const file = files[i];
      console.log('[DND] dropped file:', file.name, file.size, 'bytes');

      const reader = new FileReader();
      reader.onload = function (evt) {
        const data = new Uint8Array(evt.target.result);

        // MEMFS path
        const path = DROP_PATH + '/' + file.name;

        try {
          FS.writeFile(path, data);
          console.log('[DND] saved to MEMFS:', path);

          // Push SDL_DROPFILE event
          const pathPtr = cstr(path);
          Module._sdl_dropfile(pathPtr);
          _free(pathPtr);

        } catch (err) {
          console.error('[DND] MEMFS write failed:', err);
        }
      };

      reader.readAsArrayBuffer(file);
    }
  }, false);
};
