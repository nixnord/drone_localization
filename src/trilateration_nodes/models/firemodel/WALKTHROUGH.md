# Porting the Fire Model from Gazebo Classic to Gazebo Harmonic

This document walks through every step taken to create `fire_gz_harmonic` — a Gazebo Harmonic-compatible fire simulation model — based on the existing `fire_model` built for Gazebo Classic.

---

## 1. Investigating the Classic `fire_model`

The starting point was the `models/fire_model/` directory, which contained:

```
models/fire_model/
├── model.config                        # Model metadata (SDF 1.6)
├── model.sdf                           # SDF model definition
├── meshes/
│   ├── fire2.dae                       # COLLADA mesh (used by model.sdf)
│   ├── fire.dae
│   ├── fire1.dae
│   ├── fire.stl
│   └── Firedrop.stl
└── materials/
    ├── scripts/
    │   └── fire.material               # Ogre material script
    └── textures/
        ├── fire.jpg                    # Fire texture (443 KB)
        └── fire_color.png              # Fire colour palette (88 KB)
```

### Key observations

| File | Purpose |
|---|---|
| `model.sdf` | Static model with a single link. Uses `fire2.dae` mesh at 2× scale for both collision and visual geometry. |
| `fire.material` | An **Ogre 1.x material script** defining material `fire/object` — applies `fire.jpg` as a texture with `receive_shadows off`. |
| `model.sdf` `<visual>` | References the Ogre script via `<material><script>` tags pointing to the `materials/scripts` and `materials/textures` directories. |
| `fire2.dae` | A COLLADA file exported from Blender — a box-shaped mesh with UV coordinates for texture mapping. |

### The problem

Gazebo Harmonic (the modern Gazebo, formerly Ignition) **does not support Ogre material scripts** (the `<script>` tag inside `<material>`). It uses a new rendering pipeline based on Ogre 2.x with **Physically Based Rendering (PBR)**. This means:

- `<material><script>...</script></material>` → **not supported**
- Must use `<material><pbr><metal>...</metal></pbr></material>` instead
- Static meshes alone cannot simulate flickering fire — Gazebo Harmonic provides a native **`<particle_emitter>`** system for this

---

## 2. Researching Gazebo Harmonic Requirements

The following changes were needed based on the Gazebo Harmonic SDF specification:

### Material system

| Gazebo Classic | Gazebo Harmonic |
|---|---|
| Ogre 1.x `<script>` referencing `.material` files | PBR `<pbr><metal><albedo_map>` in SDF |
| Textures resolved via Ogre resource paths | Textures referenced as relative paths from model root |
| No emissive support in basic scripts | `<emissive>` colour tag for self-illumination |

### Particle emitter system

Gazebo Harmonic provides a built-in `<particle_emitter>` SDF element (child of `<link>`) that can simulate fire, fog, smoke, etc. This requires the world to load the plugin:

```xml
<plugin filename="gz-sim-particle-emitter-system"
        name="gz::sim::systems::ParticleEmitter"/>
```

The reference model used for study was the [Fog Generator](https://app.gazebosim.org/OpenRobotics/fuel/models/Fog%20Generator) from Gazebo Fuel, which uses a box-type particle emitter with PBR albedo map and colour range image.

### Asset compatibility

- **COLLADA (`.dae`) meshes** are supported natively by Gazebo Harmonic — no conversion needed
- **JPEG/PNG textures** are supported natively — no conversion needed
- **URI format change**: Classic used `model://fire_model/meshes/fire2.dae`; Harmonic supports relative paths like `meshes/fire2.dae`

---

## 3. Creating the New Model

### 3.1 Directory structure

```bash
mkdir -p models/fire_gz_harmonic/meshes
mkdir -p models/fire_gz_harmonic/materials/textures
```

### 3.2 Copying reusable assets

The mesh and textures are format-compatible with Harmonic, so they were copied directly:

```bash
cp fire_model/meshes/fire2.dae       fire_gz_harmonic/meshes/
cp fire_model/materials/textures/fire.jpg        fire_gz_harmonic/materials/textures/
cp fire_model/materials/textures/fire_color.png  fire_gz_harmonic/materials/textures/
```

> **Note:** The Ogre material script (`fire.material`) was **not** copied — it is not supported by Harmonic and its functionality is replaced by PBR tags in `model.sdf`.

### 3.3 Writing `model.config`

A standard model metadata file referencing SDF 1.6:

```xml
<?xml version="1.0"?>
<model>
  <name>fire_gz_harmonic</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>name@mail.com</email>
  </author>
  <description>
    A fire model for Gazebo Harmonic with particle emitter and PBR materials
  </description>
</model>
```

### 3.4 Writing `model.sdf`

This is the core of the port. The SDF was built with three components inside a single static link:

#### A) Collision geometry (preserved from Classic)

```xml
<collision name="collision">
  <geometry>
    <mesh>
      <scale>2 2 2</scale>
      <uri>meshes/fire2.dae</uri>
    </mesh>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.2</mu>
        <mu2>1.2</mu2>
      </ode>
    </friction>
  </surface>
</collision>
```

**Change from Classic:** URI changed from `model://fire_model/meshes/fire2.dae` to the relative path `meshes/fire2.dae`.

#### B) Visual geometry with PBR material (replaces Ogre script)

```xml
<visual name="visual">
  <geometry>
    <mesh>
      <scale>2 2 2</scale>
      <uri>meshes/fire2.dae</uri>
    </mesh>
  </geometry>
  <material>
    <ambient>0.8 0.25 0.0 1</ambient>
    <diffuse>1.0 0.35 0.0 1</diffuse>
    <specular>1.0 0.6 0.1 1</specular>
    <emissive>1.0 0.4 0.05 1</emissive>
    <pbr>
      <metal>
        <albedo_map>materials/textures/fire.jpg</albedo_map>
        <metalness>0.0</metalness>
        <roughness>1.0</roughness>
      </metal>
    </pbr>
  </material>
</visual>
```

**What changed and why:**
- The entire `<script>` block was removed
- `<pbr><metal><albedo_map>` replaces the Ogre material — it maps `fire.jpg` as the surface texture
- `metalness` set to `0.0` (fire is not metallic) and `roughness` to `1.0` (non-reflective)
- `<emissive>` colour added so the fire mesh glows even without external light
- Warm `<ambient>`, `<diffuse>`, and `<specular>` colours set as fallback for sensors/flat shading

#### C) Particle emitter (new — animated flame effect)

```xml
<particle_emitter name="fire_emitter" type="point">
  <emitting>true</emitting>
  <pose>0 0 0.3 0 0 0</pose>
  <size>0.5 0.5 0.5</size>
  <particle_size>0.8 0.8 0.8</particle_size>
  <lifetime>2</lifetime>
  <min_velocity>0.3</min_velocity>
  <max_velocity>0.8</max_velocity>
  <rate>15</rate>
  <scale_rate>1.0</scale_rate>
  <material>
    <diffuse>1.0 0.5 0.0</diffuse>
    <specular>1.0 0.3 0.0</specular>
    <pbr>
      <metal>
        <albedo_map>materials/textures/fire.jpg</albedo_map>
      </metal>
    </pbr>
  </material>
  <color_range_image>materials/textures/fire_color.png</color_range_image>
</particle_emitter>
```

**Design choices:**
- **`type="point"`** — particles emit from a single point, simulating a fire source
- **`pose` offset `z=0.3`** — particles start slightly above the mesh base
- **`lifetime=2`** seconds — short-lived particles for a fast-flickering fire look
- **`min/max_velocity`** — upward drift to simulate rising flames
- **`rate=15`** — 15 particles/second for a moderate flame density
- **`scale_rate=1.0`** — particles grow as they rise, simulating flame spread
- **`fire.jpg`** reused as the particle albedo texture
- **`fire_color.png`** used as `<color_range_image>` to vary particle colour over lifetime (yellow → orange → red)

---

## 4. Verification

### XML validation

Both files passed `xmllint --noout` validation:

```
✓ model.config — valid XML
✓ model.sdf    — valid XML
```

### Asset path verification

All paths referenced in `model.sdf` were confirmed to exist:

```
✓ meshes/fire2.dae                    (6,587 bytes)
✓ materials/textures/fire.jpg         (443,030 bytes)
✓ materials/textures/fire_color.png   (88,304 bytes)
```

---

## 5. Final File Listing

```
models/fire_gz_harmonic/
├── model.config                  # Model metadata
├── model.sdf                     # SDF with PBR material + particle emitter
├── WALKTHROUGH.md                # This document
├── meshes/
│   └── fire2.dae                 # COLLADA mesh (from fire_model)
└── materials/
    └── textures/
        ├── fire.jpg              # Albedo map + particle texture
        └── fire_color.png        # Particle colour range image
```

---

## 6. Usage

### World plugin requirement

Your world SDF must include the particle emitter system:

```xml
<plugin filename="gz-sim-particle-emitter-system"
        name="gz::sim::systems::ParticleEmitter"/>
```

### Including the model

```xml
<include>
  <uri>model://fire_gz_harmonic</uri>
  <pose>0 0 0 0 0 0</pose>
</include>
```

### Launching

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/path/to/models
gz sim -r your_world.sdf
```

### Runtime control

You can toggle the fire on/off at runtime via Gazebo Transport:

```bash
# Turn off
gz topic -t /model/fire_gz_harmonic/link/link/particle_emitter/fire_emitter/cmd \
  -m gz.msgs.ParticleEmitter -p 'emitting: {data: false}'

# Turn on
gz topic -t /model/fire_gz_harmonic/link/link/particle_emitter/fire_emitter/cmd \
  -m gz.msgs.ParticleEmitter -p 'emitting: {data: true}'
```
