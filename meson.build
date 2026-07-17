project(
  'nvidia-gpu-sensors',
  'c',
  version : '0.1.0',
  license : 'MIT',
  meson_version : '>=0.60.0',
  default_options : [
    'c_std=gnu11',
    'warning_level=2',
    'optimization=2',
  ],
)

tools = [
  'nvidia-gpu-sensors',  # combined sensor readout
]

foreach t : tools
  executable(t, t + '.c', install : true)
endforeach

install_subdir('docs', install_dir : get_option('datadir') / 'doc' / meson.project_name())
