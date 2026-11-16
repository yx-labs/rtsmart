"""Run the real PCM driver/CherryUSB URB layout with a deterministic fake HCD.

No real USB controller, DMA or scheduling deadlines are exercised.
"""
import os
from pathlib import Path
import runpy
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
DRIVER = HERE.parents[1]
USB = DRIVER.parent / 'CherryUSB'


def run():
    with tempfile.TemporaryDirectory() as temp:
        root = Path(temp)
        # The kernel's public HAL headers are target-specific. Shim only the
        # scheduler/DFS side, using real USBH descriptors and HCD URB structs.
        for name in ('dfs_file.h', 'dfs_posix.h', 'lwp.h', 'lwp_user_mm.h',
                     'rtthread.h', 'rtdevice.h', 'rthw.h', 'ipc/workqueue.h'):
            header = root / name
            header.parent.mkdir(exist_ok=True)
            header.write_text('#include "shim.h"\n')
        config = root / 'config.bin'
        config.write_bytes(runpy.run_path(str(HERE.parent / 'test_profile.py'))['CONFIG'])
        executable = root / 'pcm-test'
        include = [root, HERE, USB, USB / 'core', USB / 'common', USB / 'class/hub', DRIVER]
        args = [os.environ.get('CC', 'cc'), '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                '-DCHERRY_USB_HC_DRV_DWC2']
        for path in include:
            args.extend(['-I', str(path)])
        args += [str(HERE / 'runtime_test.c'), str(DRIVER / 'uac1_profile.c'), '-o', str(executable)]
        subprocess.run(args, check=True)
        subprocess.run([str(executable), str(config)], check=True)


if __name__ == '__main__':
    run()
