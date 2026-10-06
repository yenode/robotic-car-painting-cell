import numpy as np
from scipy.spatial.transform import Rotation


def read_obj(file_path):
    vertices = []
    triangles = []
    
    lines = file_path.read_text().splitlines()
    for line in lines:
        tokens = line.strip().split()
        if not tokens:
            continue
        
        if tokens[0] == 'v':
            v_coords = [float(tokens[1]), float(tokens[2]), float(tokens[3])]
            vertices.append(v_coords)
            
        elif tokens[0] == 'f':
            face_indices = []
            for token in tokens[1:]:
                idx = int(token.split('/')[0])
                if idx > 0:
                    idx = idx - 1
                else:
                    idx = len(vertices) + idx
                face_indices.append(idx)
            
            for i in range(1, len(face_indices) - 1):
                triangles.append([face_indices[0], face_indices[i], face_indices[i + 1]])
                
    return np.asarray(vertices, dtype=np.float64), np.asarray(triangles, dtype=np.int32)


def ray_hit(vertices, faces, origin, direction):
    direction = direction / np.linalg.norm(direction)
    triangles = vertices[faces]
    
    v0 = triangles[:, 0]
    v1 = triangles[:, 1]
    v2 = triangles[:, 2]
    
    edge1 = v1 - v0
    edge2 = v2 - v0
    
    h = np.cross(direction, edge2)
    determinant = np.sum(edge1 * h, axis=1)
    
    valid_mask = np.abs(determinant) > 1e-10
    inv_det = np.zeros_like(determinant)
    inv_det[valid_mask] = 1.0 / determinant[valid_mask]
    
    s = origin - v0
    u = inv_det * np.sum(s * h, axis=1)
    
    q = np.cross(s, edge1)
    v = inv_det * np.dot(q, direction)
    distance = inv_det * np.sum(edge2 * q, axis=1)
    
    valid_mask &= (u >= -1e-8) & (v >= -1e-8) & (u + v <= 1.0 + 1e-8) & (distance > 0)
    
    if not np.any(valid_mask):
        raise ValueError(f"Ray from {origin} in direction {direction} did not hit the car body mesh.")
    
    hit_idx = np.argmin(np.where(valid_mask, distance, np.inf))
    hit_point = origin + distance[hit_idx] * direction
    
    normal = np.cross(edge1[hit_idx], edge2[hit_idx])
    norm_val = np.linalg.norm(normal)
    if norm_val > 1e-12:
        normal = normal / norm_val
        
    if np.dot(normal, direction) > 0:
        normal = -normal
        
    return hit_point, normal


def tool_quaternion(surface_normal):
    z_axis = -surface_normal / np.linalg.norm(surface_normal)
    
    ref = np.array([1.0, 0.0, 0.0])
    x_axis = ref - z_axis * np.dot(ref, z_axis)
    
    if np.linalg.norm(x_axis) < 0.1:
        ref = np.array([0.0, 1.0, 0.0])
        x_axis = ref - z_axis * np.dot(ref, z_axis)
        
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    
    rot_matrix = np.column_stack([x_axis, y_axis, z_axis])
    return Rotation.from_matrix(rot_matrix).as_quat()


def mesh_clearance(vertices, faces, query_points):
    triangles = vertices[faces]
    v0, v1, v2 = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    
    def line_segment_distance(pt, p_start, p_end):
        edge = p_end - p_start
        edge_len_sq = np.sum(edge * edge, axis=1)
        t = np.clip(np.sum((pt - p_start) * edge, axis=1) / np.maximum(edge_len_sq, 1e-12), 0.0, 1.0)
        proj = p_start + t[:, None] * edge
        return np.linalg.norm(pt - proj, axis=1)

    min_distances = []
    e01, e02 = v1 - v0, v2 - v0
    normals = np.cross(e01, e02)
    norm_sq = np.sum(normals * normals, axis=1)
    valid_normals = norm_sq > 1e-20

    for pt in np.asarray(query_points):
        dot_p = np.sum((pt - v0) * normals, axis=1) / np.where(valid_normals, norm_sq, 1.0)
        proj_p = pt - dot_p[:, None] * normals
        rel_p = proj_p - v0
        
        d00 = np.sum(e01 * e01, axis=1)
        d01 = np.sum(e01 * e02, axis=1)
        d11 = np.sum(e02 * e02, axis=1)
        d20 = np.sum(rel_p * e01, axis=1)
        d21 = np.sum(rel_p * e02, axis=1)
        
        denom = d00 * d11 - d01 * d01
        valid_denom = np.abs(denom) > 1e-20
        
        v = np.zeros(len(faces))
        w = np.zeros(len(faces))
        v[valid_denom] = (d11[valid_denom] * d20[valid_denom] - d01[valid_denom] * d21[valid_denom]) / denom[valid_denom]
        w[valid_denom] = (d00[valid_denom] * d21[valid_denom] - d01[valid_denom] * d20[valid_denom]) / denom[valid_denom]
        
        is_inside = valid_denom & (v >= -1e-10) & (w >= -1e-10) & (v + w <= 1.0 + 1e-10)
        
        dist_edges = np.minimum.reduce((
            line_segment_distance(pt, v0, v1),
            line_segment_distance(pt, v1, v2),
            line_segment_distance(pt, v2, v0)
        ))
        
        dist_plane = np.abs(dot_p) * np.sqrt(norm_sq)
        dist_edges[is_inside] = dist_plane[is_inside]
        min_distances.append(dist_edges.min())
        
    return np.asarray(min_distances)


def make_strokes(vertices, faces, config):
    strokes = []
    car_offset = np.asarray(config.get('car_xyz', [0.0, 0.0, 0.0]))
    standoff = config.get('standoff', 0.25)
    
    for panel_name, patch_info in config['patches'].items():
        for row_idx, row_val in enumerate(patch_info['rows']):
            sample_points = np.linspace(patch_info['span'][0], patch_info['span'][1], 9)
            
            if row_idx % 2 == 1:
                sample_points = sample_points[::-1]
                
            pts_list = []
            normals_list = []
            
            for x_val in sample_points:
                if patch_info['ray_axis'] == 'z':
                    origin = np.array([x_val, row_val, 3.0])
                    direction = np.array([0.0, 0.0, -1.0])
                else:
                    origin = np.array([x_val, -3.0, row_val])
                    direction = np.array([0.0, 1.0, 0.0])
                    
                hit_pt, hit_norm = ray_hit(vertices, faces, origin, direction)
                pts_list.append(hit_pt)
                normals_list.append(hit_norm)
                
            avg_normal = np.mean(normals_list, axis=0)
            avg_normal = avg_normal / np.linalg.norm(avg_normal)
            
            dot_products = np.clip(np.asarray(normals_list) @ avg_normal, -1.0, 1.0)
            max_dev_deg = float(np.rad2deg(np.arccos(dot_products)).max())
            
            if max_dev_deg > 20.0:
                raise ValueError(f"{panel_name} row {row_idx}: Surface curvature exceeds 20 deg threshold ({max_dev_deg:.1f} deg)")
                
            surface_points = np.asarray(pts_list) + car_offset
            tcp_positions = surface_points + standoff * avg_normal
            orientation_quat = tool_quaternion(avg_normal)
            
            stroke_entry = {
                'name': f"{panel_name}_{row_idx + 1}",
                'panel': panel_name,
                'surface': surface_points,
                'normal': avg_normal,
                'positions': tcp_positions,
                'quaternion': orientation_quat,
                'normal_error_deg': max_dev_deg
            }
            strokes.append(stroke_entry)
            
    return strokes
