import metavision_hal
device_num = 0
for device_id in metavision_hal.DeviceDiscovery.list():
    device = metavision_hal.DeviceDiscovery.open(device_id)
    if not device.get_i_trigger_in().enable(0):
        print("The device " + device_id + " open trigger FAIL")
    else:
        print("The device " + device_id + " open trigger SUCC")
        device_num += 1
print("%d device(s) have set as trigger in mode" % device_num)
