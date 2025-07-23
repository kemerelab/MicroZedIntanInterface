import vitis
import glob

client = vitis.create_client()
client.set_workspace(path="vitis_workspace")

advanced_options = client.create_advanced_options_dict(dt_overlay="0")


platform = client.create_platform_component(name = "klab-platform",
        hw_design = "./vivado_project/klab_project.xsa",
        os = "standalone",
        cpu = "ps7_cortexa9_0",
        domain_name = "standalone_ps7_cortexa9_0",
        generate_dtb = False,
        advanced_options = advanced_options,
        compiler = "gcc")

domain = platform.get_domain(name='standalone_ps7_cortexa9_0')
domain.set_config('lib', lib_name='xiltimer', param='XILTIMER_tick_timer', value='ps7_scutimer_0')
domain.set_lib('lwip220')
domain.set_config('lib', lib_name='lwip220', param='lwip220_no_sys_no_timers', value='false')

app = client.create_app_component(name="klab-firmware",platform = "./vitis_workspace/klab-platform/export/klab-platform/klab-platform.xpfm",
        domain = "standalone_ps7_cortexa9_0")


app = client.get_component(name="klab-firmware")
status = app.import_files(from_loc="firmware", files=['src', 'include'], is_skip_copy_sources=True)

app.set_app_config('USER_INCLUDE_DIRECTORIES','../../../firmware/include')


# After creating the app:

# Append custom definitions
# with open(cmake_path, "a") as f:
#     f.write("""
# # === Custom UART and Memory Mapping ===
# set(UARTPS_NUM_DRIVER_INSTANCES "ps7_uart_1")
# set(UARTPS0_PROP_LIST "0xe0001000")
# list(APPEND TOTAL_UARTPS_PROP_LIST UARTPS0_PROP_LIST)
# set(IOMODULE_NUM_DRIVER_INSTANCES "")
# set(UARTLITE_NUM_DRIVER_INSTANCES "")
# set(UARTNS550_NUM_DRIVER_INSTANCES "")
# set(UARTPSV_NUM_DRIVER_INSTANCES "")
# """)


platform.build()

app.build()

vitis.dispose()

