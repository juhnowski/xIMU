f = open("/dev/ttyUSB0",'r')
fw = open("incoming.dat",'w+',0)


line = f.readline()
fw.write('____________________________________________\n')
while line:
    print (line),
    line = f.readline()

