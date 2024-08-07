import bpy
from map import Map, Cuboid

class RRTVisualization:
    def __init__(self):
        pass
    
    def clear_scene(self):
        for myCol in bpy.data.collections:
            if myCol.name.startswith('RRT'):
                for obj in myCol.objects:
                    bpy.data.objects.remove(obj, do_unlink=True)

    def join_selected_obj(self, final_obj_name):
        bpy.ops.object.convert(target='MESH')
        with bpy.context.temp_override(active_object=bpy.context.active_object, selected_objects=bpy.context.selected_objects):
            bpy.ops.object.join()
        bpy.context.active_object.name = final_obj_name
        # bpy.ops.object.select_all(action='DESELECT')

        for block in bpy.data.meshes:
            if block.users == 0:
                bpy.data.meshes.remove(block)

    def CollectionExists(self, collection_name):
        collectionFound = False

        for myCol in bpy.data.collections:
            if myCol.name == collection_name:
                collectionFound = True
                break
                
        return collectionFound, myCol
    
    def MakeNewCollection(self, collection_name):
        myCol = bpy.data.collections.new(collection_name)
        bpy.context.scene.collection.children.link(myCol) #Creates a new collection
        return myCol

    def DrawEdges(self, start_vertices, end_vertices, material, ObjName, shaft_radius = 0.02):

        collection_exists, collection = self.CollectionExists(ObjName)
        if not collection_exists:
            collection = self.MakeNewCollection(ObjName)

        bpy.context.active_object.select_set(False)
        bpy.ops.object.select_all(action='DESELECT')
        for i in range(0, len(start_vertices)):
            
            arrow_mesh = bpy.data.meshes.new(ObjName)
            arrow_object = bpy.data.objects.new(ObjName, arrow_mesh)
            collection.objects.link(arrow_object)
            bpy.context.view_layer.objects.active = arrow_object

            # arrow = bpy.context.active_object
            arrow_geo_node = arrow_object.modifiers.new("Arrow_Geo", "NODES")
            arrow_geo_node.node_group = bpy.data.node_groups['Edge']

            arrow_geo_node['Input_2'] = start_vertices[i] #start
            arrow_geo_node['Input_3'] = end_vertices[i] #end
            arrow_geo_node['Input_4'] = 8 #resolution
            arrow_geo_node['Input_5'] = shaft_radius #Shaft Radius
            arrow_geo_node['Input_6'] = 0.05 #Head Radius
            arrow_geo_node['Input_7'] = 0.2 #Head Length
            arrow_geo_node['Input_9'] = material

            bpy.context.active_object.select_set(True)
        
        if len(start_vertices):
            self.join_selected_obj(ObjName)
        return bpy.context.active_object
    
    def Drawvertices(self, vertices_coords, material, ObjName, radius=0.1, resolution = 2):
        collection_exists, collection = self.CollectionExists(ObjName)
        if not collection_exists:
            collection = self.MakeNewCollection(ObjName)

        bpy.context.active_object.select_set(False)

        vertices_mesh = bpy.data.meshes.new(ObjName)
        vertices_object = bpy.data.objects.new(ObjName, vertices_mesh)

        vertices_mesh.from_pydata(vertices_coords,[],[])

        collection.objects.link(vertices_object)
        bpy.context.view_layer.objects.active = vertices_object

        vertices = bpy.context.active_object
        vertices_geo_node = vertices.modifiers.new("vertices_Geo", "NODES")
        vertices_geo_node.node_group = bpy.data.node_groups['Vertices']

        vertices_geo_node['Input_2'] = radius
        vertices_geo_node['Input_4'] = resolution
        vertices_geo_node['Input_5'] = material

        bpy.context.active_object.select_set(True)
        bpy.ops.object.convert(target='MESH')
        bpy.context.active_object.name = ObjName
        bpy.ops.object.select_all(action='DESELECT')
        # bpy.ops.object.editmode_toggle()
        # bpy.ops.object.editmode_toggle()
        return bpy.context.active_object
    
    def visualize(self, rrt_planner, join_mesh = True, key_frame = 0, num_visible_frames = 1):
        vertices = rrt_planner.get_vertices()
        start_vertices, end_vertices = rrt_planner.get_edges()
        if rrt_planner.found:
            path_vertices, path_start_vertices, path_end_vertices = rrt_planner.retrieve_path()
        if join_mesh:
            bpy.ops.object.select_all(action='DESELECT')
            collection_name = "RRT_visualization"
            collection_exists, collection = self.CollectionExists(collection_name)
            if not collection_exists:
                collection = self.MakeNewCollection(collection_name)
            
            objs = []
            obj1 = self.Drawvertices(vertices, bpy.data.materials['NodeMat'], collection_name)
            objs.append(obj1)

            obj2 = self.DrawEdges(start_vertices,end_vertices, bpy.data.materials['ArrowMat'], collection_name)
            objs.append(obj2)

            obj3 = self.Drawvertices([[rrt_planner.start.x, rrt_planner.start.y, rrt_planner.start.z]],\
                            bpy.data.materials['StartMat'],collection_name,\
                            radius = 0.5)
            objs.append(obj3)

            obj4 = self.Drawvertices([[rrt_planner.goal.x, rrt_planner.goal.y, rrt_planner.goal.z]],\
                            bpy.data.materials['EndMat'],\
                            collection_name,\
                            radius = 0.5)
            objs.append(obj4)

            if rrt_planner.found:
                obj5 = self.Drawvertices(path_vertices,\
                                bpy.data.materials['FinalPathMat'],\
                                collection_name,\
                                radius = 0.2)
                objs.append(obj5)
                
                obj6 = self.DrawEdges(path_start_vertices,\
                                path_end_vertices,\
                                bpy.data.materials['FinalPathMat'],\
                                collection_name,\
                                shaft_radius = 0.1)
                objs.append(obj6)
            
            for obj in objs:
                obj.select_set(True)

            self.join_selected_obj("RRT_visualization_step")
            # bpy.context.active_object.hide_set(True)
            self.set_visiable_frames([bpy.context.active_object], key_frame, key_frame + num_visible_frames)
            bpy.ops.object.select_all(action='DESELECT')

        else:
            objs = []
            bpy.ops.object.select_all(action='DESELECT')
            # self.Drawvertices(vertices, bpy.data.materials['NodeMat'], "RRT_sampled_points")
            # bpy.context.active_object.hide_viewport = False
            # self.DrawEdges(start_vertices,end_vertices, bpy.data.materials['ArrowMat'], "RRT_edges")
            # bpy.context.active_object.hide_viewport = False
            obj1 = self.Drawvertices([[rrt_planner.start.x, rrt_planner.start.y, rrt_planner.start.z]],\
                            bpy.data.materials['StartMat'],"RRT_Start",\
                            radius = 0.5)
            self.set_visiable_frames([obj1],key_frame, key_frame + 1000)
            # objs.append(obj1)
            obj2 = self.Drawvertices([[rrt_planner.goal.x, rrt_planner.goal.y, rrt_planner.goal.z]],\
                            bpy.data.materials['EndMat'],\
                            "RRT_Goal",\
                            radius = 0.5)
            self.set_visiable_frames([obj2], key_frame, key_frame + 1000)
            # objs.append(obj2)
            obj3 = self.Drawvertices(path_vertices,\
                            bpy.data.materials['FinalPathMat'],\
                            "RRT_final_path_vertices",\
                            radius = 0.2)
            self.set_visiable_frames([obj3], key_frame, key_frame + 1000)
            obj4 = self.DrawEdges(path_start_vertices,\
                            path_end_vertices,\
                            bpy.data.materials['FinalPathMat'],\
                            "RRT_final_path",\
                            shaft_radius = 0.1)
            self.set_visiable_frames([obj4], key_frame, key_frame + num_visible_frames)
            # objs.append(obj4)
            # bpy.ops.object.select_all(action='DESELECT')
            # for obj in objs:
            #     obj.select_set(True)
            # self.join_selected_obj("RRT_visualization")
            # bpy.context.active_object.hide_set(True)
            # bpy.context.active_object.hide_render = True
            # bpy.context.active_object.keyframe_insert(data_path="hide_render", frame = key_frame)
            # bpy.context.active_object.hide_viewport = True
            # bpy.context.active_object.keyframe_insert(data_path="hide_viewport", frame = key_frame)
            # bpy.context.active_object.hide_render = False
            # bpy.context.active_object.keyframe_insert(data_path="hide_render", frame = key_frame+40)
            # bpy.context.active_object.hide_viewport = False
            # bpy.context.active_object.keyframe_insert(data_path="hide_viewport", frame = key_frame + 40)
            # bpy.context.active_object.hide_render = True
            # bpy.context.active_object.keyframe_insert(data_path="hide_render", frame = key_frame+40)
            # bpy.context.active_object.hide_viewport = True
            # bpy.context.active_object.keyframe_insert(data_path="hide_viewport", frame = key_frame + 40)
            bpy.ops.object.select_all(action='DESELECT')

    def set_visiable_frames(self,objs, frame_start, frame_end):
        for obj in objs:
            obj.hide_render = True
            obj.keyframe_insert(data_path="hide_render", frame = frame_start)
            obj.hide_viewport = True
            obj.keyframe_insert(data_path="hide_viewport", frame = frame_start)
            obj.hide_render = False
            obj.keyframe_insert(data_path="hide_render", frame = frame_start + 1)
            obj.hide_viewport = False
            obj.keyframe_insert(data_path="hide_viewport", frame = frame_start + 1)
            obj.hide_render = True
            obj.keyframe_insert(data_path="hide_render", frame = frame_end+1)
            obj.hide_viewport = True
            obj.keyframe_insert(data_path="hide_viewport", frame = frame_end+1)
            

class MapVisualization:

    def __init__(self, map):
        self.map = map
        # self.load_map(self.map_file)

    def CollectionExists(self, collection_name):
        collectionFound = False

        for myCol in bpy.data.collections:
            if myCol.name == collection_name:
                collectionFound = True
                break
                
        return collectionFound, myCol
    
    def MakeNewCollection(self, collection_name):
        myCol = bpy.data.collections.new(collection_name)
        bpy.context.scene.collection.children.link(myCol) #Creates a new collection
        return myCol
    
    def clear_scene(self):
        # bpy.ops.object.editmode_toggle()
        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.object.select_by_type(type='MESH')
        bpy.ops.object.delete()

        for block in bpy.data.meshes:
            if block.users == 0:
                bpy.data.meshes.remove(block)
        for myCol in bpy.data.collections:
            if myCol.name.startswith('Map'):
                for obj in myCol.objects:
                    bpy.data.objects.remove(obj, do_unlink=True)
        for mat in bpy.data.materials:
            if mat.users == 0:
                bpy.data.materials.remove(mat)

    def create_boundary_cuboid(self, boundaryCuboid):
        collection_exists, collection = self.CollectionExists("Map")
        if not collection_exists:
            collection = self.MakeNewCollection("Map")

        width = boundaryCuboid.xmax - boundaryCuboid.xmin
        depth = boundaryCuboid.ymax - boundaryCuboid.ymin
        height = boundaryCuboid.zmax - boundaryCuboid.zmin

        bpy.ops.mesh.primitive_cube_add(size=1, location=((boundaryCuboid.xmin + boundaryCuboid.xmax) / 2,\
                                                            (boundaryCuboid.ymin + boundaryCuboid.ymax) / 2,\
                                                            (boundaryCuboid.zmin + boundaryCuboid.zmax) / 2))

        bpy.context.view_layer.objects.active = bpy.context.object
        bpy.context.active_object.name = "Boundary"

        collection.objects.link(bpy.context.object)
        bpy.context.scene.collection.objects.unlink(bpy.context.object)

        cuboid = bpy.context.object
        cuboid.scale.x = width
        cuboid.scale.y = depth
        cuboid.scale.z = height

        mat = bpy.data.materials.new(name="CuboidMaterial")
        mat.diffuse_color = (0, 0, 0, 0.344)
        mat.blend_method = 'BLEND'
        cuboid.data.materials.append(mat)

### function for bloating obsatcles
    # def create_bloated_obstacle_cuboid(self, obstacle_cuboid):
    #     collection_exists, collection = self.CollectionExists("Map")
    #     if not collection_exists:
    #         collection = self.MakeNewCollection("Map")

    #     width = obstacle_cuboid.xmax - obstacle_cuboid.xmin
    #     depth = obstacle_cuboid.ymax - obstacle_cuboid.ymin
    #     height = obstacle_cuboid.zmax - obstacle_cuboid.zmin

    #     bpy.ops.mesh.primitive_cube_add(size=1, location=((obstacle_cuboid.xmin + obstacle_cuboid.xmax) / 2,\
    #                                                         (obstacle_cuboid.ymin + obstacle_cuboid.ymax) / 2,\
    #                                                         (obstacle_cuboid.zmin + obstacle_cuboid.zmax) / 2))
    #     bpy.context.view_layer.objects.active = bpy.context.object
    #     bpy.context.active_object.name = "Obstacle"

    #     collection.objects.link(bpy.context.object)
    #     bpy.context.scene.collection.objects.unlink(bpy.context.object)

    #     cuboid = bpy.context.object
    #     cuboid.scale.x = width  
    #     cuboid.scale.y = depth
    #     cuboid.scale.z = height

    #     mat = bpy.data.materials.new(name="BlockMaterial")
    #     mat.diffuse_color = (obstacle_cuboid.r / 255, obstacle_cuboid.g / 255, obstacle_cuboid.b / 255, 1.0)
    #     mat.blend_method = 'BLEND'
    #     mat.use_backface_culling = True
    #     cuboid.data.materials.append(mat)
    #     # collection.objects.link(bpy.context.active_object)

    def create_obstacle_cuboid(self, obstacle_cuboid):
        collection_exists, collection = self.CollectionExists("Map")
        if not collection_exists:
            collection = self.MakeNewCollection("Map")

        width = obstacle_cuboid.xmax - obstacle_cuboid.xmin
        depth = obstacle_cuboid.ymax - obstacle_cuboid.ymin
        height = obstacle_cuboid.zmax - obstacle_cuboid.zmin

        bpy.ops.mesh.primitive_cube_add(size=1, location=((obstacle_cuboid.xmin + obstacle_cuboid.xmax) / 2,\
                                                            (obstacle_cuboid.ymin + obstacle_cuboid.ymax) / 2,\
                                                            (obstacle_cuboid.zmin + obstacle_cuboid.zmax) / 2))
        bpy.context.view_layer.objects.active = bpy.context.object
        bpy.context.active_object.name = "Obstacle"

        collection.objects.link(bpy.context.object)
        bpy.context.scene.collection.objects.unlink(bpy.context.object)

        cuboid = bpy.context.object
        cuboid.scale.x = width  
        cuboid.scale.y = depth
        cuboid.scale.z = height

        mat = bpy.data.materials.new(name="BlockMaterial")
        mat.diffuse_color = (obstacle_cuboid.r / 255, obstacle_cuboid.g / 255, obstacle_cuboid.b / 255, 1.0)
        mat.blend_method = 'BLEND'
        mat.use_backface_culling = True
        cuboid.data.materials.append(mat)
        # collection.objects.link(bpy.context.active_object)

    def create_buffered_obstacle(self, obstacle_cuboid):
        collection_exists, collection = self.CollectionExists("Map")
        if not collection_exists:
            collection = self.MakeNewCollection("Map")

        width = obstacle_cuboid.xmax - obstacle_cuboid.xmin
        depth = obstacle_cuboid.ymax - obstacle_cuboid.ymin
        height = obstacle_cuboid.zmax - obstacle_cuboid.zmin

        bpy.ops.mesh.primitive_cube_add(size=1, location=((obstacle_cuboid.xmin + obstacle_cuboid.xmax) / 2,\
                                                            (obstacle_cuboid.ymin + obstacle_cuboid.ymax) / 2,\
                                                            (obstacle_cuboid.zmin + obstacle_cuboid.zmax) / 2))
        bpy.context.view_layer.objects.active = bpy.context.object
        bpy.context.active_object.name = "Buffered_Obstacle"

        collection.objects.link(bpy.context.object)
        bpy.context.scene.collection.objects.unlink(bpy.context.object)

        cuboid = bpy.context.object
        cuboid.scale.x = width + self.map.buffer
        cuboid.scale.y = depth + self.map.buffer
        cuboid.scale.z = height + self.map.buffer

        mat = bpy.data.materials.new(name="BlockMaterial")
        mat.diffuse_color = (obstacle_cuboid.r / 255, obstacle_cuboid.g / 255, obstacle_cuboid.b / 255, 0.25)
        mat.blend_method = 'BLEND'
        mat.use_backface_culling = True
        cuboid.data.materials.append(mat)

    def visualize_map(self):
        bpy.ops.object.select_all(action='DESELECT')
        self.create_boundary_cuboid(self.map.boundary)

        for obstacle in self.map.obstacles:
            self.create_obstacle_cuboid(obstacle)
            self.create_buffered_obstacle(obstacle)
        bpy.ops.object.select_all(action='DESELECT')

class TrajVisualization:
    def __init__(self):
        self.collection_name = 'Trajectory'
        pass

    def CollectionExists(self, collection_name):
        collectionFound = False

        for myCol in bpy.data.collections:
            if myCol.name == collection_name:
                collectionFound = True
                break
                
        return collectionFound, myCol
    
    def MakeNewCollection(self, collection_name):
        myCol = bpy.data.collections.new(collection_name)
        bpy.context.scene.collection.children.link(myCol) #Creates a new collection
        return myCol
    
    def clear_scene(self):
        # bpy.ops.object.editmode_toggle()
        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.object.select_by_type(type='MESH')
        bpy.ops.object.delete()

        for block in bpy.data.meshes:
            if block.users == 0:
                bpy.data.meshes.remove(block)
        for myCol in bpy.data.collections:
            if myCol.name.startswith('Trajectory'):
                for obj in myCol.objects:
                    bpy.data.objects.remove(obj, do_unlink=True)
        for mat in bpy.data.materials:
            if mat.users == 0:
                bpy.data.materials.remove(mat)

    def visualize(self, points, dt, sim_start_time, material, radius=0.2, resolution = 3):
        collection_exists, collection = self.CollectionExists(self.collection_name)
        if not collection_exists:
            collection = self.MakeNewCollection(self.collection_name)

        bpy.context.active_object.select_set(False)

        vertices_mesh = bpy.data.meshes.new(self.collection_name)
        vertices_object = bpy.data.objects.new(self.collection_name, vertices_mesh)

        vertices_mesh.from_pydata(points,[],[])

        collection.objects.link(vertices_object)
        bpy.context.view_layer.objects.active = vertices_object

        vertices = bpy.context.active_object
        vertices_geo_node = vertices.modifiers.new("trajectory_Geo", "NODES")
        vertices_geo_node.node_group = bpy.data.node_groups['Trajectory']

        vertices_geo_node['Input_2'] = radius
        vertices_geo_node['Input_4'] = resolution
        vertices_geo_node['Input_5'] = material
        vertices_geo_node['Input_6'] = dt
        vertices_geo_node['Input_7'] = sim_start_time

        # bpy.context.active_object.select_set(True)
        # bpy.ops.object.convert(target='MESH')
        # bpy.context.active_object.name = self.collection_name
        # bpy.ops.object.select_all(action='DESELECT')
        # bpy.ops.object.editmode_toggle()
        # bpy.ops.object.editmode_toggle()
        return bpy.context.active_object