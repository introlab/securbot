from pythonwifi.iwlibs import Wireless 
wifi = Wireless('wlan0') 

while(1):
	stats = wifi.getStatistics()
	print stats[1].getSignallevel() - 256


