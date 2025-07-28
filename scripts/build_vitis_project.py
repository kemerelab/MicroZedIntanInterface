import vitis
import glob

client = vitis.create_client()
client.set_workspace(path="vitis_workspace")

platform = client.get_component(name="klab-platform")
platform.build()

app = client.get_component(name="klab-firmware")
app.build()

app = client.get_component(name="klab-firmware-core1")
app.build()

vitis.dispose()

