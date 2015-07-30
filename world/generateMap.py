import Image, ImageDraw

size = 500;
img = Image.open('blank.png') # create a new black image
pixels = img.load() # create the pixel map

draw = ImageDraw.Draw(img)
draw.line((0, 0) + img.size, fill=128)
draw.line((250, 0, 250, 400), fill=0, width=10)
del draw

#orchards = 3
#orchardLoc = 0

#spacing = size/orchards

#for i in range(img.size[0]):    # for every pixel:
#    if orchardLoc == i:
#    for j in range(img.size[1]):
#        pixels[i,j] = (0, 0, 100) # set the colour accordingly
   # orchardLoc += spacing


print("test")

img.show()
img.save("testGen.png")
