# Copyright 2024 INRAE, French National Research Institute for Agriculture, Food and Environment
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import bpy
import mathutils
import os
import math
import random
import glob as _glob

import numpy as np
from PIL import Image

from . import geometry_nodes
from . import config as cfg_module


def create_blender_context(env_path=None, env_rotation_deg=0.0):
    remove_all()
    create_collections()
    geometry_nodes.create_all_node_group()
    create_environment(env_path, env_rotation_deg)

    bpy.context.scene.render.engine = "CYCLES"
    bpy.context.scene.cycles.device = "GPU"

    # enable scene lights for material preview
    bpy.data.screens["Layout"].areas[3].spaces[0].shading.use_scene_lights = True


def remove_all():
    for _, object in bpy.data.objects.items():
        bpy.data.objects.remove(object, do_unlink=True)
    for _, collection in bpy.data.collections.items():
        bpy.data.collections.remove(collection)


def create_collections():
    generated = bpy.data.collections.new("generated")
    resources = bpy.data.collections.new("resources")
    plants = bpy.data.collections.new("plants")
    weeds = bpy.data.collections.new("weeds")
    stones = bpy.data.collections.new("stones")
    env = bpy.data.collections.new("env")
    scene = bpy.context.scene.collection

    scene.children.link(env)
    scene.children.link(resources)
    scene.children.link(generated)
    resources.children.link(plants)
    resources.children.link(weeds)
    resources.children.link(stones)

    view_layer = bpy.context.scene.view_layers["ViewLayer"]
    view_layer.layer_collection.children["resources"].hide_viewport = True
    resources.hide_render = True


def create_camera(look_at: mathutils.Vector):
    camera_pos = mathutils.Vector((-13.0, look_at.y, 6.0))
    look_dir = camera_pos - look_at
    look_quaternion = look_dir.to_track_quat("Z", "Y")

    camera_data = bpy.data.cameras.new("camera")
    camera = bpy.data.objects.new("camera", camera_data)
    camera.location = camera_pos
    camera.rotation_euler = look_quaternion.to_euler()

    bpy.data.collections["env"].objects.link(camera)

    area = next(area for area in bpy.context.screen.areas if area.type == "VIEW_3D")
    region = area.spaces[0].region_3d
    region.view_location = look_at
    region.view_distance = look_dir.length - 5.0
    region.view_rotation = look_quaternion


_DEFAULT_ENV_PATH = "assets/environments/alps_field_1k.hdr"


def create_environment(env_path=None, env_rotation_deg=0.0):
    world = bpy.context.scene.world

    world.use_nodes = True
    nodes = world.node_tree.nodes
    links = world.node_tree.links

    tex_coord = nodes.new("ShaderNodeTexCoord")
    mapping = nodes.new("ShaderNodeMapping")
    mapping.inputs["Rotation"].default_value[2] = math.radians(env_rotation_deg)
    env_tex = nodes.new("ShaderNodeTexEnvironment")

    if env_path is None:
        env_image_path = os.path.join(bpy.path.abspath("//"), _DEFAULT_ENV_PATH)
    else:
        env_image_path = env_path
    env_tex.image = bpy.data.images.load(env_image_path)

    output = nodes.get("World Output")
    bg = nodes.get("Background")
    if bg is None:
        bg = nodes.new("ShaderNodeBackground")

    links.new(tex_coord.outputs["Generated"], mapping.inputs["Vector"])
    links.new(mapping.outputs["Vector"], env_tex.inputs["Vector"])
    links.new(env_tex.outputs["Color"], bg.inputs["Color"])
    links.new(bg.outputs["Background"], output.inputs["Surface"])


def setup_camera_animation(render: cfg_module.Render, bed_end: mathutils.Vector):
    scene = bpy.context.scene
    camera_cfg = render.camera

    camera_data = bpy.data.cameras.new('RenderCamera')
    camera_data.angle = math.radians(camera_cfg.fov_deg)
    camera = bpy.data.objects.new('RenderCamera', camera_data)
    camera.rotation_euler = (
        math.radians(camera_cfg.roll_deg),
        math.radians(camera_cfg.pitch_deg),
        math.radians(camera_cfg.yaw_deg),
    )
    bpy.data.collections['env'].objects.link(camera)
    scene.camera = camera

    length = bed_end.x
    width = bed_end.y
    WIDTH_PASSES = 3  # how many times to sweep across the full field width over the run

    rand = random.Random(random.getrandbits(32))
    scene.frame_start = 1
    scene.frame_end = render.frames

    for frame in range(1, render.frames + 1):
        t = (frame - 1) / max(1, render.frames - 1)
        x = t * length

        phase = (t * WIDTH_PASSES) % 1.0
        tri = 1 - abs(2 * phase - 1)   # triangle wave: 0 -> 1 -> 0
        y = tri * width

        if camera_cfg.y_jitter != 0.0:
            y += rand.uniform(-camera_cfg.y_jitter, camera_cfg.y_jitter)
            y = max(0.0, min(width, y))

        camera.location = (x, y, camera_cfg.height)
        camera.keyframe_insert(data_path='location', frame=frame)


def _quantize_masks(masks_dir: str, label_colors: cfg_module.LabelColors):
    class_colors = np.array([
        label_colors.background,
        label_colors.crop,
        label_colors.weed,
    ], dtype=np.int32)
    for path in _glob.glob(os.path.join(masks_dir, '*.png')):
        img = Image.open(path).convert('RGB')
        arr = np.array(img, dtype=np.int32)
        dists = np.sum((arr[:, :, None, :] - class_colors[None, None, :, :]) ** 2, axis=-1)
        nearest = class_colors[np.argmin(dists, axis=-1)]
        Image.fromarray(nearest.astype(np.uint8)).save(path)


def render_animation(render: cfg_module.Render, labeled: bool):
    scene = bpy.context.scene
    sub_dir = 'masks' if labeled else 'images'
    scene.render.filepath = f'//{render.directory}/{sub_dir}/frame_'
    scene.render.image_settings.file_format = 'PNG' if labeled else 'JPEG'

    if labeled:
        scene.render.engine = 'BLENDER_EEVEE'

         # added eevee path for speedup
        eevee_speed_settings = {
            'taa_render_samples': 1,
            'use_raytracing': False,
            'use_shadows': False,
            'use_volumetric_lights': False,
            'use_volumetric_shadows': False,
            'use_bloom': False,
            'use_ssr': False,
            'use_ssr_refraction': False,
            'use_soft_shadows': False,
            'shadow_pool_size': '512',
        }
        for prop_name, value in eevee_speed_settings.items():
            if hasattr(scene.eevee, prop_name):
                setattr(scene.eevee, prop_name, value)

        # path end

        scene.render.filter_size = 0.0
        scene.render.image_settings.color_mode = 'RGB'
        world = scene.world
        if world and world.use_nodes:
            bg_node = world.node_tree.nodes.get('Background')
            if bg_node:
                bg_node.inputs['Color'].default_value = (0.0, 0.0, 0.0, 1.0)
        scene.view_settings.view_transform = 'Standard'
    else:
        scene.render.engine = 'CYCLES'
        scene.cycles.device = render.cycles_device
        scene.cycles.samples = render.samples
        scene.render.image_settings.quality = 100

    scene.render.resolution_x = render.resolution_x
    scene.render.resolution_y = render.resolution_y

    bpy.ops.render.render(animation=True)

    if labeled:
        masks_dir = os.path.join(bpy.path.abspath("//"), render.directory, 'masks')
        _quantize_masks(masks_dir, render.label_colors)
