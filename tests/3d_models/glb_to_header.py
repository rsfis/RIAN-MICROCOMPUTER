import base64
import numpy as np
from pygltflib import GLTF2
from PIL import Image
import io
import os

def extract_accessor_data(gltf, accessor_index, bin_data):
    accessor = gltf.accessors[accessor_index]
    buffer_view = gltf.bufferViews[accessor.bufferView]
    start = buffer_view.byteOffset or 0
    end = start + buffer_view.byteLength
    dtype = np.float32

    if accessor.componentType == 5123:
        dtype = np.uint16
    elif accessor.componentType == 5125:
        dtype = np.uint32

    return np.frombuffer(bin_data[start:end], dtype=dtype)

def extract_texture_data(gltf, image, bin_data):
    if image.uri:
        # Caso textura venha em base64 (data URI)
        if image.uri.startswith("data:"):
            tex_data = base64.b64decode(image.uri.split(",")[1])
        else:
            with open(image.uri, "rb") as f:
                tex_data = f.read()
    else:
        # Textura embutida no buffer
        buffer_view = gltf.bufferViews[image.bufferView]
        start = buffer_view.byteOffset or 0
        end = start + buffer_view.byteLength
        tex_data = bin_data[start:end]
    return tex_data

def format_array(values, per_line=9, fmt="{}"):
    """Formata um array com N valores por linha."""
    lines = []
    for i in range(0, len(values), per_line):
        chunk = values[i:i + per_line]
        lines.append("    " + ", ".join(fmt.format(v) for v in chunk))
    return ",\n".join(lines)

def glb_to_header(glb_path, output_dir):
    gltf = GLTF2().load(glb_path)
    bin_data = gltf.binary_blob()
    model_name = os.path.splitext(os.path.basename(glb_path))[0]
    os.makedirs(output_dir, exist_ok=True)

    for mesh_index, mesh in enumerate(gltf.meshes):
        mesh_name = f"{model_name}_{mesh_index}"
        header_path = os.path.join(output_dir, f"{mesh_name}.h")

        header = "#pragma once\n#include <vector>\n#include <cstdint>\n\n"

        primitive = mesh.primitives[0]

        # === Vértices ===
        accessor_pos = gltf.accessors[primitive.attributes.POSITION]
        buffer_view = gltf.bufferViews[accessor_pos.bufferView]
        start = buffer_view.byteOffset or 0
        end = start + buffer_view.byteLength
        vertices = np.frombuffer(bin_data[start:end], dtype=np.float32)

        header += f"static const float {mesh_name}_vertices[] = {{\n"
        header += format_array([f"{v:.3f}f" for v in vertices], 9)
        header += "\n};\n\n"

        # === Índices ===
        accessor_indices = gltf.accessors[primitive.indices]
        buffer_view = gltf.bufferViews[accessor_indices.bufferView]
        start = buffer_view.byteOffset or 0
        end = start + buffer_view.byteLength
        dtype = np.uint16 if accessor_indices.componentType == 5123 else np.uint32
        indices = np.frombuffer(bin_data[start:end], dtype=dtype)

        header += f"static const unsigned int {mesh_name}_indices[] = {{\n"
        header += format_array([f"{int(i)}" for i in indices], 9)
        header += "\n};\n\n"

        # === UVs ===
        if primitive.attributes.TEXCOORD_0 is not None:
            accessor_uv = gltf.accessors[primitive.attributes.TEXCOORD_0]
            buffer_view = gltf.bufferViews[accessor_uv.bufferView]
            start = buffer_view.byteOffset or 0
            end = start + buffer_view.byteLength
            uvs = np.frombuffer(bin_data[start:end], dtype=np.float32)

            header += f"static const float {mesh_name}_uvs[] = {{\n"
            header += format_array([f"{uv:.3f}f" for uv in uvs], 9)
            header += "\n};\n\n"
        else:
            header += f"static const float {mesh_name}_uvs[] = {{}};\n\n"

        # === Textura ===
        tex_bytes = []
        tex_width, tex_height = 0, 0

        if gltf.images:
            image = gltf.images[0]
            tex_data = extract_texture_data(gltf, image, bin_data)
            img = Image.open(io.BytesIO(tex_data)).convert("RGBA")
            tex_bytes = list(img.tobytes())
            tex_width, tex_height = img.size

            header += f"static const unsigned char {mesh_name}_texture[] = {{\n"
            header += format_array([f"{b}" for b in tex_bytes], 9)
            header += "\n};\n\n"
            header += f"static const int {mesh_name}_texWidth = {tex_width};\n"
            header += f"static const int {mesh_name}_texHeight = {tex_height};\n\n"
        else:
            header += f"static const unsigned char {mesh_name}_texture[] = {{}};\n"
            header += f"static const int {mesh_name}_texWidth = 0;\n"
            header += f"static const int {mesh_name}_texHeight = 0;\n\n"

        with open(header_path, "w") as f:
            f.write(header)

        print(f"[OK] Gerado: {header_path}")

if __name__ == "__main__":
    glb_to_header(input("INPUT PATH: "), input("OUTPUT PATH: "))
