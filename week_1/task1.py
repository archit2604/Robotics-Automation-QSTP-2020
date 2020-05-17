import math
#function to convert polar coordinates to cartesian
def polar2cartesian(r,t):
    x=r*math.cos(t)
    y=r*math.sin(t)
    return x,y
#function to convert cartesian coordinates to polar
def cartesian2polar(x,y):
    r=math.sqrt((x*x)+(y*y))
    t=math.atan(y/x)
    #the following lines deal with to adjust atan values according to quadrants
    if x<0 and y>0:
        t=math.pi+t
    elif x<0 and y<0:
        t=t-math.pi
    return r,t
#the main function
if __name__=="__main__":
    #loop for continous use of converter
    while(True):    
        print("Coordinate Converter:")
        print("1.Convert from Cartesian to Polar Coordinates")
        print("2.Convert from Polar to Cartesian Coordinates")
        op=input("Enter 1 or 2:")
        #block for converting cartesian coordinates to polar
        if op=='1':
            x=float(input("Enter the x-coordinate:"))
            y=float(input("Enter the y-coordinate:"))
            r,t=cartesian2polar(x,y)
            print("The corresponding Polar Coordinates are:")
            print("r:"+str(r))
            print("Theta(in radians):"+str(t))
        #block for converting polar coordinates to cartesian
        elif op=='2':
            r=float(input("Enter the radial coordinate:"))
            t=float(input("Enter the angular coordinate(in radians):"))
            x,y=polar2cartesian(r,t)
            print("The corresponding Cartesian Coordinates are:")
            print("x:"+str(x))
            print("y:"+str(y)) 
            #block for dealing with invaid inputs
        else:
            print("Invalid Input!")
        #block to ask the user if they want to use the converter again    
        chk=input("Do you want to use the converter again?(y/n):")
        if chk=='n':
            break
