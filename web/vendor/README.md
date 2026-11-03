# Local visualization dependencies

Three.js 0.180.0 (`three.module.js`, `three.core.js`, `OrbitControls.js`,
`STLLoader.js`, `THREE-LICENSE.txt`) copied from the installed
`/home/mine/a3u-pose-viewer/web/vendor/` on 2026-11-01.
MIT license: see `THREE-LICENSE.txt`. No CDN or network assets are used.

Table dimensions and geometry follow that viewer's `native_scene.py`:
2.74 × 1.525 m, 0.76 m height, 0.1525 m net. The implementation here is
independent and uses the tabletop-centred coordinate frame in `docs/frames.md`.
Robot meshes are served from this repository's existing robot_model submodule;
they have their own license (`assets/robot_model/LICENSE.txt`).
