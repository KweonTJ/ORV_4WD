#!/usr/bin/env python3
"""Build static RViz assets from the supplied V3 STEP and procedural rubber wheels.

Offline authoring dependency: cadquery-ocp==7.8.1.1.post1, numpy.
The installed ROS package uses the exported meshes, not these dependencies.
"""

import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


COLORS = {
    'paint': (0.19, 0.205, 0.22),
    'motor': (0.43, 0.45, 0.47),
    'fastener': (0.66, 0.68, 0.70),
    'window': (0.69, 0.72, 0.73),
    'rubber': (0.070, 0.074, 0.079),
    'tread': (0.14, 0.145, 0.15),
    'rim': (0.22, 0.23, 0.24),
    'hub': (0.29, 0.30, 0.31),
}


class Mesh:
    def __init__(self):
        self.groups = defaultdict(list)

    def triangle(self, a, b, c, material):
        self.groups[material].append((a, b, c))

    def quad(self, a, b, c, d, material):
        self.triangle(a, b, c, material)
        self.triangle(a, c, d, material)

    def ring(self, profile, material, segments=128):
        """Revolve a closed (axial y, radius) profile about +Y."""
        def point(y, radius, angle):
            return (radius*math.cos(angle), y, radius*math.sin(angle))
        for i in range(segments):
            a, b = 2*math.pi*i/segments, 2*math.pi*(i+1)/segments
            for j, (y, r) in enumerate(profile):
                yy, rr = profile[(j+1) % len(profile)]
                self.quad(point(y, r, a), point(yy, rr, a),
                          point(yy, rr, b), point(y, r, b), material)

    def prism(self, polygon, y0, y1, material):
        """Extrude a convex polygon in the XZ plane along Y."""
        low = [(x, y0, z) for x, z in polygon]
        high = [(x, y1, z) for x, z in polygon]
        for i in range(1, len(polygon)-1):
            self.triangle(low[0], low[i], low[i+1], material)
            self.triangle(high[0], high[i+1], high[i], material)
        for i in range(len(polygon)):
            j = (i+1) % len(polygon)
            self.quad(low[j], low[i], high[i], high[j], material)

    def tread_bar(self, angle, y0, y1, u0, u1, radius=0.044, depth=0.0018):
        """Raised rectangle in the unwrapped tread plane, wrapped around +Y."""
        def p(u, y, r):
            a = angle + u/radius
            return (r*math.cos(a), y, r*math.sin(a))
        points = [(u0,y0),(u1,y0),(u1,y1),(u0,y1)]
        lo = [p(u,y,radius-depth) for u,y in points]
        hi = [p(u,y,radius) for u,y in points]
        self.quad(*hi[::-1], 'tread')
        for i in range(4):
            j=(i+1)%4
            self.quad(lo[j],lo[i],hi[i],hi[j],'tread')

    def write(self, path):
        ns = 'http://www.collada.org/2005/11/COLLADASchema'
        ET.register_namespace('', ns)
        def el(parent, tag, attrib=None, text=None):
            node=ET.SubElement(parent, '{'+ns+'}'+tag, attrib or {})
            if text is not None: node.text=text
            return node
        root=ET.Element('{'+ns+'}COLLADA',version='1.4.1')
        asset=el(root,'asset'); el(asset,'unit',{'name':'meter','meter':'1'}); el(asset,'up_axis',text='Z_UP')
        effects=el(root,'library_effects'); materials=el(root,'library_materials')
        geometries=el(root,'library_geometries'); scenes=el(root,'library_visual_scenes')
        scene=el(scenes,'visual_scene',{'id':'Scene','name':'Scene'})
        report={}
        for material, triangles in self.groups.items():
            xyz=np.asarray(triangles,dtype=np.float64)
            n=np.cross(xyz[:,1]-xyz[:,0],xyz[:,2]-xyz[:,0])
            lengths=np.linalg.norm(n,axis=1)
            xyz=xyz[lengths>1e-15]; n=n[lengths>1e-15]; lengths=lengths[lengths>1e-15]
            n=n/lengths[:,None]
            effect=el(effects,'effect',{'id':material+'-fx'})
            tech=el(el(effect,'profile_COMMON'),'technique',{'sid':'common'})
            phong=el(tech,'phong')
            for channel, values in [('ambient',tuple(v*.6 for v in COLORS[material])),('diffuse',COLORS[material]),('specular',(0.12,)*3)]:
                el(el(phong,channel),'color',text=' '.join(map(str,(*values,1))))
            el(el(phong,'shininess'),'float',text='24')
            mat=el(materials,'material',{'id':material,'name':material})
            el(mat,'instance_effect',{'url':'#'+material+'-fx'})
            geometry=el(geometries,'geometry',{'id':material+'-geometry'})
            mesh=el(geometry,'mesh')
            for suffix, data in [('positions',xyz.reshape(-1,3)),('normals',n)]:
                sid=material+'-'+suffix
                source=el(mesh,'source',{'id':sid})
                el(source,'float_array',{'id':sid+'-array','count':str(data.size)},
                   ' '.join(f'{v:.8g}' for v in data.ravel()))
                accessor=el(el(source,'technique_common'),'accessor',{'source':'#'+sid+'-array','count':str(len(data)),'stride':'3'})
                for name in ('X','Y','Z'): el(accessor,'param',{'name':name,'type':'float'})
            vertices=el(mesh,'vertices',{'id':material+'-vertices'})
            el(vertices,'input',{'semantic':'POSITION','source':'#'+material+'-positions'})
            tris=el(mesh,'triangles',{'count':str(len(xyz)),'material':material})
            el(tris,'input',{'semantic':'VERTEX','source':'#'+material+'-vertices','offset':'0'})
            el(tris,'input',{'semantic':'NORMAL','source':'#'+material+'-normals','offset':'1'})
            el(tris,'p',text=' '.join(f'{3*i+j} {i}' for i in range(len(xyz)) for j in range(3)))
            node=el(scene,'node',{'id':material+'-node'})
            inst=el(node,'instance_geometry',{'url':'#'+material+'-geometry'})
            bind=el(el(inst,'bind_material'),'technique_common')
            el(bind,'instance_material',{'symbol':material,'target':'#'+material})
            report[material]=len(xyz)
        el(el(root,'scene'),'instance_visual_scene',{'url':'#Scene'})
        ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
        return report


def chassis(step_path, cache_path):
    from OCP.BRep import BRep_Builder, BRep_Tool
    from OCP.BRepTools import BRepTools
    from OCP.TopoDS import TopoDS_Shape, TopoDS, TopoDS_Compound
    from OCP.STEPControl import STEPControl_Reader
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopExp import TopExp_Explorer
    from OCP.BRepBndLib import BRepBndLib
    from OCP.Bnd import Bnd_Box
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopLoc import TopLoc_Location
    builder=BRep_Builder()
    if cache_path and cache_path.exists():
        shape=TopoDS_Shape(); BRepTools.Read_s(shape,str(cache_path),builder)
    else:
        reader=STEPControl_Reader(); reader.ReadFile(str(step_path)); reader.TransferRoots()
        shape=reader.OneShape()
        if cache_path: BRepTools.Write_s(shape,str(cache_path))
    compound=TopoDS_Compound(); builder.MakeCompound(compound)
    selected=[]; iterator=TopExp_Explorer(shape,TopAbs_FACE)
    # This supplied STEP is a fused Mecanum assembly, not a named-part assembly.
    # Wheels lie outside these native Y planes; retain the chassis and inner motors.
    while iterator.More():
        face=TopoDS.Face_s(iterator.Current()); box=Bnd_Box(); BRepBndLib.Add_s(face,box)
        bounds=box.Get()
        # The STEP has a narrow center bumper; the rubber-wheel photos show
        # a full-width perforated bracket, reconstructed below instead.
        narrow_bumper = bounds[0]>136.8 and bounds[3]<140.0 and bounds[5]<169.0
        if bounds[1]>138 and bounds[4]<299 and not narrow_bumper:
            selected.append((face,bounds)); builder.Add(compound,face)
        iterator.Next()
    print(f'Tessellating {len(selected)} chassis/motor faces',flush=True)
    BRepMesh_IncrementalMesh(compound,0.15,False,0.25,True).Perform()
    batches=[]
    for face,bounds in selected:
        location=TopLoc_Location(); triangulation=BRep_Tool.Triangulation_s(face,location)
        if triangulation is None: continue
        transform=location.Transformation()
        vertices=np.array([tuple(triangulation.Node(i).Transformed(transform).Coord()) for i in range(1,triangulation.NbNodes()+1)])
        indices=np.array([triangulation.Triangle(i).Get() for i in range(1,triangulation.NbTriangles()+1)])-1
        if face.Orientation()==TopAbs_REVERSED: indices=indices[:,[0,2,1]]
        # Most outer panels are powder coated. Exposed motor cans are metallic.
        dx,dy,dz=(bounds[i+3]-bounds[i] for i in range(3))
        x,y,z=((bounds[i]+bounds[i+3])/2 for i in range(3))
        is_motor=(dx<38 and 146<z<184 and (abs(x+85.97)<19 or abs(x-106.12)<19)
                  and 158<y<280)
        batches.append((vertices[indices], 'motor' if is_motor else 'paint'))
    all_points=np.concatenate([a.reshape(-1,3) for a,_ in batches])
    lo,hi=all_points.min(axis=0),all_points.max(axis=0)
    center=(lo+hi)/2
    scale=np.array([.290,.155,.067])/(hi-lo)
    mesh=Mesh()
    for points, material in batches:
        mesh.groups[material].extend((points-center)*scale)
    # Frosted insert behind the actual cut-outs in the sloping front frame.
    # Source plane: x+z ~= 333.98 mm; recess the insert 1.2 mm behind it.
    raw=np.array([[137.7,150.0,195.08],[158.2,150.0,174.58],
                  [158.2,287.6,174.58],[137.7,287.6,195.08]])
    panel=(raw-center)*scale
    mesh.quad(*panel,'window'); mesh.quad(*panel[::-1],'window')
    # Front-frame bolt heads, placed in source coordinates then fitted with the shell.
    normal=np.array([1.,0.,1.])/math.sqrt(2)
    tangent=np.array([1.,0.,-1.])/math.sqrt(2)
    along=np.array([0.,1.,0.])
    for x,y in [(138.8,146.0),(138.8,291.6),(155.0,149.0),(155.0,288.6)]:
        c=np.array([x,y,333.98-x])+normal*.7
        for i in range(24):
            a,b=2*math.pi*i/24,2*math.pi*(i+1)/24
            r=2.6
            p=c+r*(math.cos(a)*along+math.sin(a)*tangent)
            q=c+r*(math.cos(b)*along+math.sin(b)*tangent)
            mesh.triangle((c+normal*1.0-center)*scale,(p+normal*1.0-center)*scale,(q+normal*1.0-center)*scale,'fastener')
            mesh.quad((p-center)*scale,(q-center)*scale,(q+normal*1.0-center)*scale,(p+normal*1.0-center)*scale,'fastener')
    # Broad front sensor/bumper bracket seen in the user photographs.
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
    from OCP.gp import gp_Pnt, gp_Ax2, gp_Dir
    bracket=BRepPrimAPI_MakeBox(gp_Pnt(.120,-.070,-.0335),.002,.140,.032).Shape()
    for y,r in [(-.052,.007),(.052,.007),(-.033,.0015),(.033,.0015)]:
        drill=BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(.118,y,-.0145),gp_Dir(1,0,0)),r,.006).Shape()
        bracket=BRepAlgoAPI_Cut(bracket,drill).Shape()
    for y,z,w,h in [(-.0025,-.025,.005,.021),(-.010,-.0175,.020,.006),(-.0045,-.030,.009,.0025)]:
        slot=BRepPrimAPI_MakeBox(gp_Pnt(.118,y,z),.006,w,h).Shape()
        bracket=BRepAlgoAPI_Cut(bracket,slot).Shape()
    def add_shape(shape, point_map=None, reverse=False):
        BRepMesh_IncrementalMesh(shape,.00015,False,.2,True).Perform()
        faces=TopExp_Explorer(shape,TopAbs_FACE)
        while faces.More():
            face=TopoDS.Face_s(faces.Current()); loc=TopLoc_Location()
            tri=BRep_Tool.Triangulation_s(face,loc)
            if tri is not None:
                points=np.array([tuple(tri.Node(i).Transformed(loc.Transformation()).Coord()) for i in range(1,tri.NbNodes()+1)])
                if point_map: points=point_map(points)
                indices=np.array([tri.Triangle(i).Get() for i in range(1,tri.NbTriangles()+1)])-1
                if (face.Orientation()==TopAbs_REVERSED) != reverse: indices=indices[:,[0,2,1]]
                mesh.groups['paint'].extend(points[indices])
            faces.Next()
    add_shape(bracket)
    # Folded side skirts with three open ventilation slots, visible in the photos.
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace, BRepBuilderAPI_Transform
    from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
    from OCP.gp import gp_Vec, gp_Trsf, gp_Ax1
    wire=BRepBuilderAPI_MakePolygon()
    for u,v in [(-.044,0),(.038,0),(.030,.029),(-.033,.029)]: wire.Add(gp_Pnt(u,v,0))
    wire.Close()
    skirt=BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(wire.Wire()).Face(),gp_Vec(0,0,.0015)).Shape()
    for u in (-.022,-.007,.008):
        slot=BRepPrimAPI_MakeBox(gp_Pnt(u-.0015,.005,-.001),.003,.019,.004).Shape()
        rotation=gp_Trsf();rotation.SetRotation(gp_Ax1(gp_Pnt(u,.0145,0),gp_Dir(0,0,1)),-.25)
        skirt=BRepAlgoAPI_Cut(skirt,BRepBuilderAPI_Transform(slot,rotation).Shape()).Shape()
    for side in (-1,1):
        def placement(points):
            u,v,w=points.T
            return np.column_stack((u,side*(.075-.32*v+.947*w),.032-.947*v-.32*w))
        add_shape(skirt,placement,reverse=side<0)
    return mesh, {'source_bounds_mm':[lo.tolist(),hi.tolist()], 'mesh_size_m':[.290,.155,.067],
                  'source_faces':len(selected), 'source_to_mesh_scale':scale.tolist(),
                  'source_center_mm':center.tolist()}


def wheel():
    mesh=Mesh()
    # Sidewall shoulders and recessed tread bed, 88 mm outside diameter x 35 mm width.
    mesh.ring([(-.0175,.0315),(-.0175,.0375),(-.0168,.0400),(-.0148,.0422),
               (.0148,.0422),(.0168,.0400),(.0175,.0375),(.0175,.0315)],'rubber')
    for side in (-1,1):
        rim=[(side*.0160,.0305),(side*.0160,.0325),(side*.0172,.0325),(side*.0172,.0305)]
        hub=[(side*.0145,.0032),(side*.0145,.0090),(side*.0173,.0090),(side*.0173,.0032)]
        mesh.ring(rim if side>0 else rim[::-1],'rim')
        mesh.ring(hub if side>0 else hub[::-1],'hub',64)
        if side<0:
            continue
        # Swept spokes approximate the narrow multi-spoke molded wheel in the photos.
        for i in range(16):
            angle=2*math.pi*i/16
            def polar(r,a): return (r*math.cos(a),r*math.sin(a))
            polygon=[polar(.008,angle-.12),polar(.0317,angle+.17),
                     polar(.0317,angle+.24),polar(.008,angle+.12)]
            low=[(x,.0025+.010*math.hypot(x,z)/.0317,z) for x,z in polygon]
            high=[(x,y+.0025,z) for x,y,z in low]
            for j in (1,2):
                mesh.triangle(low[0],low[j],low[j+1],'rim')
                mesh.triangle(high[0],high[j+1],high[j],'rim')
            for j in range(4):
                k=(j+1)%4
                mesh.quad(low[k],low[j],high[j],high[k],'rim')
    mesh.ring([(-.0145,.0032),(-.0145,.0075),(.0145,.0075),(.0145,.0032)],'hub',64)
    # Five staggered columns of alternating hooked/C-shaped blocks form maze grooves.
    pitch=2*math.pi*.044/32
    for row in range(32):
        for col in range(5):
            angle=2*math.pi*(row+(col%2)*.5)/32
            yc=(col-2)*.0057
            u0,u1=-pitch*.39,pitch*.39
            y0,y1=yc-.00235,yc+.00235
            thick=.00115
            flip=(row+col)%2
            mesh.tread_bar(angle,y0,y0+thick,u0,u1)
            mesh.tread_bar(angle,y1-thick,y1,u0,u1)
            if flip:
                mesh.tread_bar(angle,y0,y1,u1-thick,u1)
                mesh.tread_bar(angle,yc-thick/2,yc+thick/2,u0+thick,u1-thick)
            else:
                mesh.tread_bar(angle,y0,y1,u0,u0+thick)
                mesh.tread_bar(angle,yc-thick/2,yc+thick/2,u0+thick,u1-thick)
    return mesh


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--step',type=Path,required=True)
    parser.add_argument('--cache',type=Path)
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[1]/'meshes')
    parser.add_argument('--preview-arrays',type=Path,help='Optional authoring-only NumPy preview data')
    args=parser.parse_args(); args.output.mkdir(parents=True,exist_ok=True)
    body, provenance=chassis(args.step,args.cache)
    provenance['source_file']=args.step.name
    provenance['source_sha256']=hashlib.sha256(args.step.read_bytes()).hexdigest()
    provenance['body_triangles']=body.write(args.output/'v3_chassis.dae')
    tire=wheel(); provenance['wheel_triangles']=tire.write(args.output/'rubber_wheel.dae')
    provenance['wheel_mesh_size_m']={'diameter':.088,'width':.035}
    provenance['notes']=['Chassis from user-supplied STEP, rescaled to measured length and body height.',
                         'Wheel tread, spokes, front bumper, vented side skirts, window insert and bolt heads are visual reconstructions from photos.',
                         'Body width 155 mm and wheelbase 200 mm remain unmeasured.',
                         'STEP source is a Mecanum version; original wheels are excluded.']
    (args.output/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    # Optional authoring-only arrays for independent render/geometry inspection.
    if args.preview_arrays:
        np.savez_compressed(args.preview_arrays,**{f'body_{k}':np.asarray(v) for k,v in body.groups.items()},
                            **{f'wheel_{k}':np.asarray(v) for k,v in tire.groups.items()})
    print(json.dumps(provenance,indent=2),flush=True)


if __name__=='__main__': main()
