from pythonwifi.iwlibs import Wireless 
wifi = Wireless('wlan0') 
while(1):
	Stats = wifi.getStatistics()
	level = Stats[1].getSignallevel()
	print level

