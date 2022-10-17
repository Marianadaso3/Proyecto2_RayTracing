#Mariana David
#Carne:201055

from gl import Raytracer, V3
from texture import *
from figures import *
from lights import *


width = 256
height = 256

# # Materiales
sun = Material( diffuse = (0.85, 0.45, 0), spec = 64, matType = TRANSPARENT)
moon = Material( diffuse = (0.9, 0.9, 0.9), spec = 64, matType = OPAQUE)
piedra = Material(diffuse = (0.2, 0.2, 0.2), spec = 16, matType=OPAQUE, texture = Texture('piedra.bmp'))
marble = Material(spec = 64 ,texture = Texture("marble.bmp"), matType= REFLECTIVE)
glass = Material(diffuse = (0.9, 0.9, 0.9), spec = 64, ior = 1.5, matType = TRANSPARENT)

rtx = Raytracer(width, height)
rtx.envMap = Texture("F1.bmp")

#Lights
rtx.lights.append( AmbientLight(intensity = 0.1 ))
rtx.lights.append( PointLight(point = (-1,-1,0) ))

#Figures
rtx.scene.append( Sphere(center = (0,0,-10), radius = 2, material = glass))
rtx.scene.append( Sphere(center = (-8,8,-20), radius = 0.5, material = moon))
rtx.scene.append( Sphere(center = ( 8,8,-20), radius = 1, material = sun))
for i in range(-4, 5):
    rtx.scene.append( AABB(position = (i,-((1/4)*(i**2))+3,-10), size = (1,1,1), material = marble))
for i in range(-4, 5):
    rtx.scene.append( AABB(position = (i,((1/16)*(i**2))-3,-10), size = (1,1,1), material = piedra))

#Render
rtx.glRender()
rtx.glFinish("output.bmp")