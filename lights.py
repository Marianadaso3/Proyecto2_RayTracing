#Mariana David 
#Carne: 201055 
from mymath import *
DIR_LIGHT = 0
POINT_LIGHT = 1
AMBIENT_LIGHT = 2

def reflectVector(normal, direction):
    reflect = vectDot(V3(normal[0], normal[1], normal[2]), V3(direction[0], direction[1], direction[2])) * 2
  
    reflect = multiply( V3(normal[0], normal[1], normal[2]), reflect)
    reflect = vectSubtract(V3(reflect[0], reflect[1], reflect[2]), V3(direction[0], direction[1], direction[2]))

    reflect = normalize(reflect)
    return reflect

def refractVector(normal, direction, ior):
    # Snell's Law

    # vectDot(V3(normal[0], normal[1], normal[2]), V3(direction[0], direction[1], direction[2]))
    cosi = max(-1, min(1, vectDot(V3(normal[0], normal[1], normal[2]), V3(direction[0], direction[1], direction[2]))))
    etai = 1
    etat = ior

    if cosi < 0:
        cosi = -cosi
    else:
        etai, etat = etat, etai
        normal = multiply(V3(normal[0], normal[1], normal[2]), -1)

    eta = etai / etat
    k = 1 - (eta**2) * (1 - (cosi**2) )

    if k < 0: # Total Internal Reflection
        return None


    a = multiply(V3(direction[0], direction[1], direction[2]), eta)
    b = multiply(V3(normal[0], normal[1], normal[2]), (eta * cosi - k**0.5))
    R = vectAdd(V3(a[0], a[1], a[2]), V3(b[0], b[1], b[2]))
    
    return R


def fresnel(normal, direction, ior):
    # Fresnel Equation
    cosi = max(-1, min(1, vectDot(V3(normal[0], normal[1], normal[2]), V3(direction[0], direction[1], direction[2])) ))
    etai = 1
    etat = ior

    if cosi > 0:
        etai, etat = etat, etai

    sint = etai / etat * (max(0, 1 - cosi**2) ** 0.5)


    if sint >= 1: # Total Internal Reflection
        return 1

    cost = max(0, 1 - sint**2) ** 0.5
    cosi = abs(cosi)

    Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost))
    Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost))

    return (Rs**2 + Rp**2) / 2

class PointLight(object):
    def __init__(self, point, constant = 1.0, linear = 0.1, quad = 0.05, color = (1,1,1)):
        self.point = point
        self.constant = constant
        self.linear = linear
        self.quad = quad
        self.color = color
        self.lightType = POINT_LIGHT

    def getDiffuseColor(self, intersect, raytracer):
        light_dir = vectSubtract(V3(self.point[0], self.point[1], self.point[2]), V3(intersect.point[0], intersect.point[1], intersect.point[2]))
        light_dir = normalize(light_dir)

        # att = 1 / (Kc + Kl * d + Kq * d * d)
        #attenuation = 1.0 / (self.constant + self.linear * lightDistance + self.quad * lightDistance ** 2)
        attenuation = 1.0
        intensity = vectDot(V3(intersect.normal[0], intersect.normal[1], intersect.normal[2]), V3(light_dir[0], light_dir[1], light_dir[2])) * attenuation
        intensity = float(max(0, intensity))            
                                                        
        diffuseColor = [intensity * self.color[0],
                                 intensity * self.color[1],
                                 intensity * self.color[2]]

        return diffuseColor

    def getSpecColor(self, intersect, raytracer):
        light_dir = vectSubtract(V3(self.point[0], self.point[1], self.point[2]), V3(intersect.point[0], intersect.point[1], intersect.point[2]))
        light_dir = normalize(light_dir)

        reflect = reflectVector(intersect.normal, light_dir)

        view_dir = vectSubtract(V3(raytracer.camPosition[0], raytracer.camPosition[1], raytracer.camPosition[2]), V3(intersect.point[0], intersect.point[1], intersect.point[2]))
        view_dir = normalize(view_dir)

        # att = 1 / (Kc + Kl * d + Kq * d * d)
        #attenuation = 1.0 / (self.constant + self.linear * lightDistance + self.quad * lightDistance ** 2)
        attenuation = 1.0
        spec_intensity = attenuation * max(0,vectDot(V3(reflect[0], reflect[1], reflect[2]), V3(view_dir[0], view_dir[1], view_dir[2]))) ** intersect.sceneObj.material.spec
        specColor =[spec_intensity * self.color[0],
                              spec_intensity * self.color[1],
                              spec_intensity * self.color[2]]

        return specColor

    def getShadowIntensity(self, intersect, raytracer):
        import numpy as np
        light_dir = vectSubtract(V3(self.point[0], self.point[1], self.point[2]), V3(intersect.point[0], intersect.point[1], intersect.point[2]))
        light_distance = np.linalg.norm(light_dir)
        light_dir = normalize(light_dir)

        shadow_intensity = 0
        shadow_intersect = raytracer.scene_intersect(intersect.point, light_dir, intersect.sceneObj)
        if shadow_intersect:
            if shadow_intersect.distance < light_distance:
                shadow_intensity = 1

        return shadow_intensity


class AmbientLight(object):
    def __init__(self, intensity = 0.1, color = (1,1,1)):
        self.intensity = intensity
        self.color = color
        self.lightType = AMBIENT_LIGHT

    def getDiffuseColor(self, intersect, raytracer):
        r = multiply(V3(self.color[0], self.color[1], self.color[2]), self.intensity)
        return r


    def getSpecColor(self, intersect, raytracer):
        return [0,0,0]

    def getShadowIntensity(self, intersect, raytracer):
        return 0
