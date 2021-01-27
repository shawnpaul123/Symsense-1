from tkinter import *
from PIL import ImageTk, Image
import time
import threading

root = Tk()
root.title('SymSense')
#root.iconbitmap('directory.cio')
def readTemperatureSensor():
    global my_label, l1
    temperature = 40
    l1 = Label(root,text=str(temperature)+'C')
    l1.grid(row=0,column=1)
    my_label.grid_forget()
    my_label = Label(image=image_list[5])
    my_label.grid(row=5,column=0,columnspan=3)

def temperatureSensor():
    global my_label
    my_label.grid_forget()
    my_label = Label(image=image_list[4])
    my_label.grid(row=5,column=0,columnspan=3)
    threading.Timer(3.0, readTemperatureSensor).start()

def checkCamera():
    global my_label
    my_label.grid_forget()
    my_label = Label(image=image_list[2])
    my_label.grid(row=5,column=0,columnspan=3)

def camera():
    global my_label
    my_label.grid_forget()
    my_label = Label(image=image_list[1])
    my_label.grid(row=5,column=0,columnspan=3,rowspan=3)
    threading.Timer(3.0, checkCamera).start()

def dispenseHS():
    global my_label
    my_label.grid_forget()
    my_label = Label(image=image_list[0])
    my_label.grid(row=5,column=0,columnspan=3,rowspan=3)

def openGate():
    global my_label
    my_label.grid_forget()
    my_label = Label(image=image_list[0])
    my_label.grid(row=5,column=0,columnspan=3,rowspan=3)


#for testing
b1=Button(root,text="Read Temperature Sensor",command=lambda:temperatureSensor())
temperature = 38
l1 = Label(root,text=str(temperature)+'C')
b2=Button(root,text="Start Camera",command=lambda:camera())
b3=Button(root,text="Dispense Hand Sanitizer",command=lambda:dispenseHS())
b4=Button(root,text="Open Gate",command=lambda:openGate())
button_quit = Button(root,text='Exit Program',command=root.quit)
b1.grid(row=0,column=0)
l1.grid(row=0,column=1)
b2.grid(row=1,column=0)
b3.grid(row=2,column=0)
b4.grid(row=3,column=0)
button_quit.grid(row=4,column=0)

zoom = 1.5
#multiple image size by zoom
image = Image.open('img/picamera.png')
pixels_x, pixels_y = tuple([int(zoom * x)  for x in image.size])
my_img = ImageTk.PhotoImage(Image.open('img/picamera.png').resize((pixels_x, pixels_y)))
my_img1 = ImageTk.PhotoImage(Image.open('img/camera.PNG').resize((pixels_x, pixels_y)))
my_img2 = ImageTk.PhotoImage(Image.open('img/camera_success.PNG').resize((pixels_x, pixels_y)))
my_img3 = ImageTk.PhotoImage(Image.open('img/camera_fail.PNG').resize((pixels_x, pixels_y)))
my_img4 = ImageTk.PhotoImage(Image.open('img/temperature.PNG').resize((pixels_x, pixels_y)))
my_img5 = ImageTk.PhotoImage(Image.open('img/fail.PNG').resize((pixels_x, pixels_y)))
image_list = [my_img, my_img1, my_img2, my_img3, my_img4, my_img5]

my_label = Label(image=my_img)
my_label.grid(row=5,column=0,columnspan=2,rowspan=2)
root.mainloop()
'''
while True:
    # MAIN LOOP that will replace root.mainloop
    root.update_idletasks()
    root.update()
'''
