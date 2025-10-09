# Order is important
subdir('test-utils')
subdir('circuits')
subdir('arena')
subdir('float')
subdir('json')
subdir('can-kingdom')
subdir('rover')
subdir('stm32-common')
subdir('postmaster')

common_deps += [rover_dep, can_kingdom_mayor_dep, stm32_common_dep]
app_deps = [common_deps, circuits_dep, postmaster_dep]
