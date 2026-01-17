Module.js_init_fs = function () {
  console.log('[FS] js_init_fs');

  const APP_NAME = 'applewin';
  const APP_CONFIG_DIR = '/persist/.config/' + APP_NAME;
  const DEFAULT_DIR = '/defaults/.config/' + APP_NAME;

  // 1. Mount IDBFS
  console.log('[FS] Mount /persist');
  FS.mkdir('/persist');
  FS.mount(IDBFS, {}, '/persist');
  console.log('[FS] Mounted /persist');

  // 2. Load persistent filesystem
  FS.syncfs(true, function (err) {
    if (err) {
      console.error('[FS] syncfs(load) failed', err);
      return;
    }

    // 3. Now create directories INSIDE IDBFS
    FS.mkdirTree(APP_CONFIG_DIR);

    // 4. Copy defaults if missing
    const files = ['applewin.yaml', 'imgui.ini'];

    for (const name of files) {
      const dst = APP_CONFIG_DIR + '/' + name;
      const src = DEFAULT_DIR + '/' + name;

      if (!FS.analyzePath(dst).exists) {
        console.log('[FS] Copy from:', src, 'to', dst);
        const data = FS.readFile(src);
        FS.writeFile(dst, data);
        console.log('[FS] Copied default:', name);
      }
    }

    // 5. Persist changes
    FS.syncfs(false, function () {
      console.log('[FS] Persistent FS ready');
      Module._on_fs_ready();
    });
  });
};
