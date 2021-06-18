from bluepy import btle

class MyDelegate(btle.DefaultDelegate):
  def __init__(self, params):
      btle.DefaultDelegate.__init__(self)
      # ... initialise here

  def handleNotification(self, cHandle, data):
      # ... perhaps check cHandle
      # ... process 'data'
      print ("**** data: " + data.decode("utf8"))

print ("Connecting...")
#dev = btle.Peripheral("20:91:48:BB:D3:98")
dev = btle.Peripheral("00:15:83:00:64:35")

print ("Services...")
for svc in dev.services:
  print (str(svc))

dev.setDelegate( MyDelegate("test_param") )

lightSensor = btle.UUID("0000ffe0-0000-1000-8000-00805f9b34fb")

lightService = dev.getServiceByUUID(lightSensor)
for ch in lightService.getCharacteristics():
  print (str(ch))
  print ("Props: %s", ch.propertiesToString())
  ch.write("hello!!!".encode("utf8"))
  print("READ: %s", ch.read())

print ("***********")


while True:
    if dev.waitForNotifications(1.0):
        # handleNotification() was called
        continue

    print ("Waiting...")
    # Perhaps do something else here