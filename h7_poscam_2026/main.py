import sensor, image, time, math
from pyb import Pin, Timer
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 1000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(	 False, exposure_us=1000 )
sensor.set_auto_gain(False, gain_db=25)
clock = time.clock()
from pyb import UART
uart = UART(1, 1000000, timeout_char=1000)
uart.init(1000000, bits=8, parity=None, stop=1, timeout_char=1000)
def crc16(data : bytearray, offset , length):
	if data is None or offset < 0 or offset > len(data)- 1 and offset+length > len(data):
		return 0
	crc = 0xFFFF
	for i in range(0, length):
		crc ^= data[offset + i] << 8
		for j in range(0,8):
			if (crc & 0x8000) > 0:
				crc =(crc << 1) ^ 0x1021
			else:
				crc = crc << 1
	return crc & 0xFFFF
tag_families = 0
tag_families |= image.TAG36H11
sensor.set_vflip(True)
def family_name(tag):
	if(tag.family() == image.TAG16H5):
		return "TAG16H5"
	if(tag.family() == image.TAG25H7):
		return "TAG25H7"
	if(tag.family() == image.TAG25H9):
		return "TAG25H9"
	if(tag.family() == image.TAG36H10):
		return "TAG36H10"
	if(tag.family() == image.TAG36H11):
		return "TAG36H11"
	if(tag.family() == image.ARTOOLKIT):
		return "ARTOOLKIT"
p = Pin('P8')
tim = Timer(4, freq=1000)
ch = tim.channel(2, Timer.PWM, pin=p)
ch.pulse_width_percent(50)
p1 = Pin('P9')
p1.init(Pin.IN,pull = Pin.PULL_UP)
count=0
uartBuff = []
newFrameDetected = False
newFramePos = 0
masterFrameLen=6
workMode=1
frameWidth = sensor.width()
frameHeight = sensor.height()
while(True):
	clock.tick()
	img = sensor.snapshot()
	if(workMode>0):
		tagCount=0
		for tag in img.find_apriltags(families=tag_families):
			packet=bytearray((b'\xaa\x55\x01\x11'))
			packet.append((tag.id()>>8))
			packet.append((tag.id()&0xff))
			packet.append(int(tag.cx()/frameWidth*255))
			packet.append(int(tag.cy()/frameHeight*255))
			rotationDeg = (180 * tag.rotation()) / math.pi*10
			if(rotationDeg<0):
				rotationDeg+=3600
			outputDeg = int(rotationDeg)
			packet.append((outputDeg>>8))
			packet.append((outputDeg&0xff))
			datalen = len(packet)-2;
			cs_byte = crc16(packet,2,datalen)
			packet.append(cs_byte>>8)
			packet.append(cs_byte&0xff)
			tagCount=tagCount+1
			uart.write(packet)
			print(tag.id())
	datalen = uart.any()
	if(datalen):
		if(len(uartBuff)+datalen>1000):
			uartBuff=[]
			uart.read(datalen)
			continue
		uartBuff.append(uart.read(datalen))
		for i in range(len(uartBuff)-1):
			if(uartBuff[i]==0xAA):
				if(uartBuff[i+1]==0x55):
					newFrameDetected=True
					newFramePos=i
	if(newFrameDetected):
		uartBuff=[]
		if(newFramePos<=(uartBuff.size()-6)):
			addressByte = uartBuff[newFramePos+2]
			if(addressByte==0x11):
				commandByte=uartBuff[newFramePos+3]
				if(commandByte==0x01):
					workMode=1
					sensor.set_pixformat(sensor.GRAYSCALE)
					sensor.set_framesize(sensor.QVGA)
				elif(commandByte==0x02):
					workMode=2
					sensor.set_pixformat(sensor.GRAYSCALE)
					sensor.set_framesize(sensor.QQVGA)
				else:
					workMode=0
	if(len(uartBuff)>1000):
		uartBuff=[]
	print(clock.fps())