from tkinter import *
app = Tk()
app.geometry("400x400")

canvas = Canvas(app, bg='black')
canvas.pack(anchor='nw', fill='both', expand=1)

def get_x_and_y(event):
    global lasx, lasy
    lasx, lasy = event.x, event.y

def draw_smth(event):
    global lasx, lasy
    canvas.create_line((lasx, lasy, event.x, event.y), 
                      fill='white', 
                      width=5)
    lasx, lasy = event.x, event.y
    
def clear(event):
    canvas.delete("all")

canvas.bind("<Button-1>", get_x_and_y)
canvas.bind("<B1-Motion>", draw_smth)
canvas.bind("<Button-3>", clear)
app.mainloop()