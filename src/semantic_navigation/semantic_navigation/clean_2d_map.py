#!/usr/bin/env python3
"""Clean a Nav2 PGM map or generate a neat 2D map from the Gazebo world."""

from pathlib import Path
import argparse
import math
import shutil
import sys
import xml.etree.ElementTree as ET


OCCUPIED_MAX_VALUE = 64
OCCUPIED_VALUE = 0
UNKNOWN_VALUE = 205
FREE_VALUE = 254

OBJECT_FOOTPRINTS = {
    'cafe_table': (0.9, 0.9),
    'first_2015_trash_can': (0.55, 0.55),
    'cardboard_box': (0.65, 0.65),
    'table': (1.35, 0.85),
    'cabinet': (0.9, 0.55),
    'bookshelf': (0.85, 0.4),
    'wooden_case': (0.65, 0.65),
    'grey_tote': (0.75, 0.55),
}


def read_pgm(path):
    data = Path(path).read_bytes()
    offset = 0

    def next_token():
        nonlocal offset
        while offset < len(data) and data[offset] in b' \t\r\n':
            offset += 1
        if offset < len(data) and data[offset] == ord('#'):
            while offset < len(data) and data[offset] not in b'\r\n':
                offset += 1
            return next_token()
        start = offset
        while offset < len(data) and data[offset] not in b' \t\r\n':
            offset += 1
        return data[start:offset]

    magic = next_token()
    if magic != b'P5':
        raise ValueError(f'{path} is not a binary PGM (P5) file')
    width = int(next_token())
    height = int(next_token())
    max_value = int(next_token())
    while offset < len(data) and data[offset] in b' \t\r\n':
        offset += 1
    pixels = bytearray(data[offset:])
    expected = width * height
    if len(pixels) != expected:
        raise ValueError(f'{path} has {len(pixels)} pixels, expected {expected}')
    return width, height, max_value, pixels


def write_pgm(path, width, height, max_value, pixels):
    header = f'P5\n{width} {height}\n{max_value}\n'.encode()
    Path(path).write_bytes(header + bytes(pixels))


def write_yaml(path, image_name, resolution, origin):
    text = (
        f'image: {image_name}\n'
        'mode: trinary\n'
        f'resolution: {resolution:.6f}\n'
        f'origin: [{origin[0]:.6f}, {origin[1]:.6f}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n'
    )
    Path(path).write_text(text)


def occupied_components(width, height, pixels):
    occupied = [value <= OCCUPIED_MAX_VALUE for value in pixels]
    seen = [False] * (width * height)
    components = []

    for index, is_occupied in enumerate(occupied):
        if not is_occupied or seen[index]:
            continue
        stack = [index]
        seen[index] = True
        component = []
        while stack:
            current = stack.pop()
            component.append(current)
            x = current % width
            y = current // width
            for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if 0 <= nx < width and 0 <= ny < height:
                    neighbor = ny * width + nx
                    if occupied[neighbor] and not seen[neighbor]:
                        seen[neighbor] = True
                        stack.append(neighbor)
        components.append(component)
    return components


def copy_yaml(input_yaml, output_yaml, image_name):
    text = Path(input_yaml).read_text()
    lines = []
    replaced = False
    for line in text.splitlines():
        if line.strip().startswith('image:'):
            lines.append(f'image: {image_name}')
            replaced = True
        else:
            lines.append(line)
    if not replaced:
        lines.insert(0, f'image: {image_name}')
    Path(output_yaml).write_text('\n'.join(lines) + '\n')


def parse_numbers(text):
    return [float(value) for value in text.split()]


def parse_pose(element):
    pose_element = element.find('pose')
    if pose_element is None or not pose_element.text:
        return 0.0, 0.0, 0.0
    values = parse_numbers(pose_element.text)
    x = values[0] if len(values) > 0 else 0.0
    y = values[1] if len(values) > 1 else 0.0
    yaw = values[5] if len(values) > 5 else 0.0
    return x, y, yaw


def rotate_point(x, y, yaw):
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return x * cos_yaw - y * sin_yaw, x * sin_yaw + y * cos_yaw


def box_bounds(cx, cy, sx, sy, yaw, padding=0.0):
    hx = sx / 2.0 + padding
    hy = sy / 2.0 + padding
    corners = []
    for lx, ly in ((-hx, -hy), (-hx, hy), (hx, -hy), (hx, hy)):
        rx, ry = rotate_point(lx, ly, yaw)
        corners.append((cx + rx, cy + ry))
    xs = [point[0] for point in corners]
    ys = [point[1] for point in corners]
    return min(xs), min(ys), max(xs), max(ys)


def point_in_box(px, py, cx, cy, sx, sy, yaw, padding=0.0):
    dx = px - cx
    dy = py - cy
    local_x, local_y = rotate_point(dx, dy, -yaw)
    return abs(local_x) <= sx / 2.0 + padding and abs(local_y) <= sy / 2.0 + padding


def world_to_pixel(px, py, min_x, min_y, resolution, width, height):
    mx = int((px - min_x) / resolution)
    my = int((py - min_y) / resolution)
    return mx, height - 1 - my


def draw_box(pixels, width, height, min_x, min_y, resolution, box, value, padding=0.0):
    cx, cy, sx, sy, yaw = box
    bx0, by0, bx1, by1 = box_bounds(cx, cy, sx, sy, yaw, padding)
    start_x = max(0, int((bx0 - min_x) / resolution) - 2)
    end_x = min(width - 1, int((bx1 - min_x) / resolution) + 2)
    start_y = max(0, int((by0 - min_y) / resolution) - 2)
    end_y = min(height - 1, int((by1 - min_y) / resolution) + 2)

    for mx in range(start_x, end_x + 1):
        wx = min_x + (mx + 0.5) * resolution
        for my in range(start_y, end_y + 1):
            wy = min_y + (my + 0.5) * resolution
            if point_in_box(wx, wy, cx, cy, sx, sy, yaw, padding):
                image_y = height - 1 - my
                pixels[image_y * width + mx] = value


def extract_box_collision(collision):
    geometry = collision.find('geometry')
    if geometry is None:
        return None
    box = geometry.find('box')
    if box is None:
        return None
    size = box.find('size')
    if size is None or not size.text:
        return None
    values = parse_numbers(size.text)
    if len(values) < 2:
        return None
    x, y, yaw = parse_pose(collision)
    return x, y, values[0], values[1], yaw


def extract_inline_box_model(model):
    model_x, model_y, model_yaw = parse_pose(model)
    link = model.find('link')
    if link is None:
        return None
    collision = link.find('collision')
    if collision is None:
        return None
    geometry = collision.find('geometry')
    if geometry is None:
        return None
    box = geometry.find('box')
    if box is None:
        return None
    size = box.find('size')
    if size is None or not size.text:
        return None
    values = parse_numbers(size.text)
    if len(values) < 2:
        return None
    local_x, local_y, local_yaw = parse_pose(collision)
    rotated_x, rotated_y = rotate_point(local_x, local_y, model_yaw)
    return model_x + rotated_x, model_y + rotated_y, values[0], values[1], model_yaw + local_yaw


def extract_included_model_footprint(model):
    include = model.find('include')
    if include is None:
        return None
    uri = include.find('uri')
    if uri is None or not uri.text:
        return None
    model_name = uri.text.replace('model://', '').strip()
    if model_name not in OBJECT_FOOTPRINTS:
        return None
    x, y, yaw = parse_pose(model)
    sx, sy = OBJECT_FOOTPRINTS[model_name]
    return x, y, sx, sy, yaw


def generate_world_map(world_file, output_yaml, output_pgm, resolution, padding):
    root = ET.parse(world_file).getroot()
    floors = []
    walls = []
    objects = []

    for collision in root.findall(".//collision"):
        name = collision.attrib.get('name', '')
        box = extract_box_collision(collision)
        if box is None:
            continue
        if name.startswith('floor_'):
            floors.append(box)
        elif name.startswith('wall_'):
            walls.append(box)

    for model in root.findall('.//model'):
        name = model.attrib.get('name', '')
        if name in {
            'floor_network_high_friction',
            'transparent_low_walls_merged',
            'robot_spawn_clear_area_reference',
            'semantic_furniture_and_obstacles',
            'visual_semantic_landmarks',
        }:
            continue
        footprint = extract_inline_box_model(model) or extract_included_model_footprint(model)
        if footprint is not None:
            objects.append(footprint)

    all_boxes = floors + walls + objects
    if not floors or not all_boxes:
        raise ValueError(f'No usable map geometry found in {world_file}')

    bounds = [box_bounds(*box, padding=padding) for box in all_boxes]
    min_x = min(bound[0] for bound in bounds) - padding
    min_y = min(bound[1] for bound in bounds) - padding
    max_x = max(bound[2] for bound in bounds) + padding
    max_y = max(bound[3] for bound in bounds) + padding
    width = int(math.ceil((max_x - min_x) / resolution))
    height = int(math.ceil((max_y - min_y) / resolution))
    pixels = bytearray([UNKNOWN_VALUE] * (width * height))

    for floor in floors:
        draw_box(pixels, width, height, min_x, min_y, resolution, floor, FREE_VALUE)
    for wall in walls:
        draw_box(pixels, width, height, min_x, min_y, resolution, wall, OCCUPIED_VALUE, padding=0.08)
    for obstacle in objects:
        draw_box(pixels, width, height, min_x, min_y, resolution, obstacle, OCCUPIED_VALUE, padding=0.20)

    output_yaml = Path(output_yaml)
    output_pgm = Path(output_pgm)
    output_pgm.parent.mkdir(parents=True, exist_ok=True)
    write_pgm(output_pgm, width, height, 255, pixels)
    write_yaml(output_yaml, output_pgm.name, resolution, (min_x, min_y))

    print(
        f'Generated world-refined map from {world_file}: '
        f'floors={len(floors)}, walls={len(walls)}, objects={len(objects)}, '
        f'size={width}x{height}, resolution={resolution}'
    )
    print(f'Wrote {output_yaml}')
    print(f'Wrote {output_pgm}')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-yaml', default=str(Path.home() / '.ros' / 'k12_semantic_map.yaml'))
    parser.add_argument('--input-pgm', default=str(Path.home() / '.ros' / 'k12_semantic_map.pgm'))
    parser.add_argument('--output-yaml', default=str(Path.home() / '.ros' / 'k12_semantic_map_clean.yaml'))
    parser.add_argument('--output-pgm', default=str(Path.home() / '.ros' / 'k12_semantic_map_clean.pgm'))
    parser.add_argument('--min-occupied-area', type=int, default=8)
    parser.add_argument('--world-file', default='')
    parser.add_argument('--world-resolution', type=float, default=0.05)
    parser.add_argument('--world-padding', type=float, default=0.25)
    args = parser.parse_args(argv)

    if args.world_file:
        generate_world_map(
            args.world_file,
            args.output_yaml,
            args.output_pgm,
            args.world_resolution,
            args.world_padding,
        )
        return 0

    input_yaml = Path(args.input_yaml)
    input_pgm = Path(args.input_pgm)
    output_yaml = Path(args.output_yaml)
    output_pgm = Path(args.output_pgm)
    if not input_yaml.exists() or not input_pgm.exists():
        raise FileNotFoundError('Run export_rtabmap_2d_map.launch.py before cleaning the map')

    width, height, max_value, pixels = read_pgm(input_pgm)
    components = occupied_components(width, height, pixels)
    removed = 0
    kept = 0
    for component in components:
        if len(component) < args.min_occupied_area:
            removed += len(component)
            for index in component:
                pixels[index] = FREE_VALUE
        else:
            kept += len(component)

    output_pgm.parent.mkdir(parents=True, exist_ok=True)
    write_pgm(output_pgm, width, height, max_value, pixels)
    if input_yaml.resolve() == output_yaml.resolve():
        shutil.copy2(input_yaml, output_yaml.with_suffix('.yaml.bak'))
    copy_yaml(input_yaml, output_yaml, output_pgm.name)

    print(
        f'Cleaned {input_pgm}: components={len(components)}, '
        f'removed_occupied_pixels={removed}, kept_occupied_pixels={kept}'
    )
    print(f'Wrote {output_yaml}')
    print(f'Wrote {output_pgm}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
