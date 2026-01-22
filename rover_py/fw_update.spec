# -*- mode: python ; coding: utf-8 -*-

a = Analysis(
    ['fw_update.py'],
    pathex=[],
    binaries=[],
    datas=[],
    hiddenimports=[
        'can.interfaces.kvaser',
        'can.interfaces.socketcan',
        'can.interfaces.virtual',
        'can.interfaces.pcan',
        'can.interfaces.usb2can',
        'can.interfaces.ixxat',
        'can.interfaces.vector',
        'can.interfaces.slcan',
        'can.interfaces.canalystii',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
)

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='fw_update',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
