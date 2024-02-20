import os
from .abc import *
from .io import unpack
from mathutils import Vector, Matrix, Quaternion
from enum import IntEnum

DISTRICT_187_TEST = False

# LTB Mesh Types
LTB_Type_Rigid_Mesh = 4
LTB_Type_Skeletal_Mesh = 5
LTB_Type_Vertex_Animated_Mesh = 6
LTB_Type_Null_Mesh = 7

# Data stream flags
VTX_Position      = 0x0001
VTX_Normal        = 0x0002
VTX_Colour        = 0x0004
VTX_UV_Sets_1     = 0x0010
VTX_UV_Sets_2     = 0x0020
VTX_UV_Sets_3     = 0x0040
VTX_UV_Sets_4     = 0x0080
VTX_BasisVector   = 0x0100

# Animation Compression Types
CMP_None = 0
CMP_Relevant = 1
CMP_Relevant_16 = 2
CMP_Relevant_Rot16 = 3

Invalid_Bone = 255

class VertexPropertyFormat(IntEnum):
	Float_x2=1 # Vector2f
	Float_x3=2 # Vector3f
	Float_x4=3 # Vector4f
	Byte_x4=4 # Int_x1?
	SkeletalIndex=5 # Float or Int, depending on shader defs
	Exit=17

class VertexPropertyLocation(IntEnum):
	Position=0
	BlendWeight=1
	BlendIndices=2
	Normal=3
	TexCoords=5
	Tangent=6
	Binormal=7
	Colour=10

#
# Supports Model00p v33 (FEAR)
#
class PCModel00PackedReader(object):
	def __init__(self):
		self.is_little_endian = True

		self.version = 0
		self.node_count = 0
		self.lod_count = 0
		self.string_table = ""

		# Temp until I can figure out how animations work!
		self._read_animations = True

		# Re-map our material indexes
		self.material_index_list = {}

	# Helper class for reading in mesh data
	# TODO: Need to clean up and move into a model class whenever I get to that.
	class MeshData(object):
		def __init__(self):
			self.vertex = Vector()
			self.normal = Vector()
			self.uvs = Vector() # [] when adding in multiple UV maps
			self.tangent = Vector()
			self.binormal = Vector()
			self.weight_info = []
			self.node_indexes = []
			self.colour = []

		def read(self, reader, f, vertex_format):

			tmp_pos = f.tell()

			for prop in vertex_format.properties:

				pack_str = ""
				if prop.format == VertexPropertyFormat.Float_x2:
					pack_str = "2f"
				elif prop.format == VertexPropertyFormat.Float_x3:
					pack_str = "3f"
				elif prop.format == VertexPropertyFormat.Float_x4:
					pack_str = "4f"
				elif prop.format == VertexPropertyFormat.Byte_x4:
					pack_str = "4B"
				elif prop.format == VertexPropertyFormat.SkeletalIndex:
					pack_str = "4b" # 4 bytes
				elif prop.format == VertexPropertyFormat.Exit:
					raise ValueError("Vertex property Exit found")
					continue # this /should/ never activate since we chop the final entry off when reading the descriptors
				else:
					raise ValueError("Invalid vertex property format")

				unpacked = reader._unpack(pack_str, f)

				if prop.id > 0: # handle this properly!
					print("Unhandled vertex property, id > 0")
					continue

				if prop.location == VertexPropertyLocation.Position:
					self.vertex = Vector([ unpacked[0], unpacked[1], unpacked[2]])
				elif prop.location == VertexPropertyLocation.BlendWeight:
					self.weight_info = tuple(reversed(unpacked[0:3]))
				elif prop.location == VertexPropertyLocation.BlendIndices:
					self.node_indexes = unpacked[0:3]
				elif prop.location == VertexPropertyLocation.Normal:
					self.normal = Vector([ unpacked[0], unpacked[2], unpacked[1] ])
				elif prop.location == VertexPropertyLocation.TexCoords:
					self.uvs = Vector([ unpacked[0], 1.0 - unpacked[1] ])
				elif prop.location == VertexPropertyLocation.Tangent:
					self.tangent = Vector([ unpacked[0], unpacked[2], unpacked[1] ])
				elif prop.location == VertexPropertyLocation.Binormal:
					self.binormal = Vector([ unpacked[0], unpacked[2], unpacked[1] ])
				elif prop.location == VertexPropertyLocation.Colour:
					self.colour = Vector([ unpacked[0] / 255, unpacked[1] / 255, unpacked[2] / 255, unpacked[3] / 255 ])
				else:
					raise ValueError("Unknown vertex property location")

			return self

	# Another helper class, this should be shoved into whatever I refactor into FEAR's model.
	class MeshInfo(object):
		def __init__(self):
			self.mesh_data_start = 0
			self.mesh_data_count = 0
			self.mesh_data_size = 0
			self.index_list_position = 0
			self.unk_3 = 0
			self.triangle_count = 0
			self.material_index = 0
			self.influence_count = 0
			self.vertex_format_index = 0
			self.influence_node_indexes = []

		def read(self, reader, f):
			self.mesh_data_start = reader._unpack('I', f)[0]
			self.mesh_data_count = reader._unpack('I', f)[0]
			self.mesh_data_size = reader._unpack('I', f)[0]
			self.index_list_position = reader._unpack('I', f)[0]
			self.unk_3 = reader._unpack('I', f)[0]
			self.triangle_count = reader._unpack('I', f)[0]
			self.material_index = reader._unpack('I', f)[0]
			self.influence_count = reader._unpack('I', f)[0]
			self.vertex_format_index = reader._unpack('I', f)[0]
			self.influence_node_indexes = [ reader._unpack('b', f)[0] for _ in range(self.influence_count) ]

			return self

	class VertexFormatProperty(object):
		size = 8

		def __init__(self):
			self.cont_flag = 0 # -1 = end
			# pad byte
			self.offset = 0
			self.format=-1
			self.location=-1
			self.id=0

		def read(self, reader, f):
			self.cont_flag = reader._unpack('bb', f)[0] # read 2 bytes, assume 2nd is pad unless proven otherwise
			self.offset, self.format = reader._unpack('2H', f)
			self.location, self.id = reader._unpack('2B', f)

			return self

	class VertexFormatDef(object):
		def __init__(self):
			self.size = 0
			self.properties = []

		def read(self, reader, f):
			self.size = int(reader._unpack('I', f)[0] / PCModel00PackedReader.VertexFormatProperty.size)
			self.properties = [ PCModel00PackedReader.VertexFormatProperty().read(reader, f) for _ in range(self.size) ]
			self.properties = self.properties[0:len(self.properties) - 1]

			return self

	class AnimInfo(object):
		def __init__(self):
			self.binding = None

			self.animation = Animation()

	class Physics(object):
		def __init__(self):
			self.vis_node_index = -1
			self.vis_radius = -1

			self.shape_count = -1
			self.shapes = []

			self.constraint_count = -1
			self.contraints = []

			self.weight_set_count = -1
			self.weight_sets = []

	class PhysicsShape(object):
		class HavokShape(object):
			def __init__(self):
				self.type_id = 0

				self.mass = -1
				self.density = -1
				self.radius = -1

				self.offset = Vector()
				self.orientation = Quaternion()

			def __repr__(self):
				return "HavokShape({} {} {}, {} {})".format(self.mass, self.density, self.radius, self.offset, self.orientation)

		def __init__(self):
			self.index = -1
			self.offset = -1 # Vector()
			self.orientation = Quaternion()
			self.cor = -1
			self.friction = -1
			self.collision_group = -1
			self.node_index = -1

			self.havok_shapes = []

	class PhysicsConstraint(object):
		def __init__(self):
			self.type = 0
			self.nodes = []
			self.data = []
			self.friction = 0
			self.unk_2 = 0
			self.unk_3 = 0

	#
	# Wrapper around .io.unpack that can eventually handle big-endian reads.
	#
	def _unpack(self, fmt, f):

		# Force big endian if we're not little!
		if self.is_little_endian == False:
			fmt = '>%s' % fmt

		return unpack(fmt, f)

	def _get_string_from_table(self, offset):
		value = self.string_table[offset:]

		# Okay we need to find the next null character now!
		null_terminator = -1
		for (index, char) in enumerate(value):
			if char == '\x00':
				null_terminator = index
				break

		# Make sure we actually ran through the string
		assert(null_terminator != -1)

		length = offset + null_terminator

		return self.string_table[offset:length]

	def _read_matrix(self, f):
		data = self._unpack('16f', f)
		rows = [data[0:4], data[4:8], data[8:12], data[12:16]]
		return Matrix(rows)

	def _read_short_vector(self, f):
		x,y,z = self._unpack('3h', f)
		return [x,y,z]

	def _read_vector(self, f):
		return Vector(self._unpack('3f', f))

	def _read_short_quaternion(self, f):
		x, y, z, w = self._unpack('4h', f)
		return [w,x,y,z]

	def _read_quaternion(self, f):
		x, y, z, w = self._unpack('4f', f)
		return Quaternion((w, x, y, z))

	def _read_string(self, f):
		return f.read(self._unpack('H', f)[0]).decode('ascii')

	def _read_fixed_string(self, length, f):
		return f.read(length).decode('ascii')

	def _read_weight(self, f):
		weight = Weight()
		weight.node_index = self._unpack('I', f)[0]
		weight.location = self._read_vector(f)
		weight.bias = self._unpack('f', f)[0]
		return weight

	def _read_vertex(self, f):
		vertex = Vertex()
		weight_count = self._unpack('H', f)[0]
		vertex.sublod_vertex_index = self._unpack('H', f)[0]
		vertex.weights = [self._read_weight(f) for _ in range(weight_count)]
		vertex.location = self._read_vector(f)
		vertex.normal = self._read_vector(f)
		return vertex

	def _read_face_vertex(self, f):
		face_vertex = FaceVertex()
		face_vertex.texcoord.xy = self._unpack('2f', f)
		face_vertex.vertex_index = self._unpack('H', f)[0]
		return face_vertex

	def _read_face(self, f):
		face = Face()
		face.vertices = [self._read_face_vertex(f) for _ in range(3)]
		return face

	def _read_mesh_data(self, pieces, f):
		debug_ftell = f.tell()

		_ = self._unpack('I', f)[0]
		lod_count = self._unpack('I', f)[0]
		texture_count = self._unpack('I', f)[0]

		data_length = self._unpack('I', f)[0]
		index_list_length = self._unpack('I', f)[0]

		print("Mesh Data Triangle Count: ", data_length / 64)

		# We need to read the MeshInfo data that's located AFTER the actual mesh data
		# So skip the mesh data for now...

		mesh_data_position = f.tell()

		f.seek(data_length, 1)
		f.seek(index_list_length, 1)

		vertex_format_count = self._unpack('I', f)[0]
		vertex_formats = [ self.VertexFormatDef().read(self, f) for _ in range(vertex_format_count) ]

		# Okay here's the mesh info!
		mesh_info_count = self._unpack('I', f)[0]
		mesh_info = [ self.MeshInfo().read(self, f) for _ in range(mesh_info_count) ]

		mesh_info_position = f.tell()

		# Hop back to the mesh data
		f.seek(mesh_data_position, 0)

		# NEW

		# TODO: Maybe sort info by data start?

		mesh_data_list = []
		for index in range(mesh_info_count):
			info = mesh_info[index]

			length = info.mesh_data_count
			for _ in range( length ):
				data = self.MeshData()
				mesh_data_list.append( data.read(self, f, vertex_formats[info.vertex_format_index]) )

		assert(f.tell() == (mesh_data_position + data_length))

		# END NEW

		print("Data List Length -> ", len(mesh_data_list))

		index_list = [ self._unpack('H', f)[0] for _ in range( int(index_list_length / 2) ) ]

		debug_ftell = f.tell()

		assert(f.tell() == mesh_data_position + data_length + index_list_length)

		# Annnnd hope back
		f.seek(mesh_info_position, 0)

		# End

		# Some running totals
		running_index_list_index = 0
		for index in range(len(pieces)):
			piece = pieces[index]

			for lod_index in range(len(piece.lods)):
				lod = piece.lods[lod_index]

				if lod.piece_count == 0:
					continue

				info = mesh_info[lod.piece_index_list[0]]
				running_index_list_index = info.index_list_position

				# Set the material index (for the main lod only!)
				if lod_index == 0:

					if info.material_index in self.material_index_list:
						piece.material_index = self.material_index_list[info.material_index]
					else:
						length = len(self.material_index_list)
						self.material_index_list[info.material_index] = piece.material_index = length

					#piece.material_index = info.material_index

				for vertex_index in range(info.mesh_data_start, info.mesh_data_start + info.mesh_data_count):
					mesh_data = mesh_data_list[ vertex_index ]
					vertex = Vertex()
					vertex.location = mesh_data.vertex
					vertex.normal = mesh_data.normal

					# Weight info kinda sucks, if there's only one weight it's on position 3...
					# Why? I don't know, I'm hoping I'm reading this data slightly wrong.
					#if mesh_data.weight_info[2] == 255:
					#    weight = Weight()
					index = 0
					for bias in mesh_data.weight_info:
						if bias == 0:
							continue

						weight = Weight()
						weight.bias = bias / 255
						weight.node_index = info.influence_node_indexes[ mesh_data.node_indexes[index] ]

						vertex.weights.append(weight)

						index += 1

					lod.vertices.append(vertex)

				# Holds 3 face vertices
				face = Face()

				# Tri count * 3, because we're looping through the index list for the length of tri count.
				# This may not entirely be correct, as IndexList / 3 sometimes does not equal triangle counts!
				for index in range(info.triangle_count * 3):
					face_vertex = FaceVertex()

					face_vertex.vertex_index = index_list[running_index_list_index]

					face_vertex.texcoord = mesh_data_list[face_vertex.vertex_index].uvs

					face_vertex.vertex_index -= info.mesh_data_start

					face.vertices.append(face_vertex)

					if len(face.vertices) == 3:
						lod.faces.append(face)
						face = Face()

					running_index_list_index += 1


				piece.lods[lod_index] = lod

		return pieces

	def _read_lod(self, f):
		lod = LOD()

		lod.distance = self._unpack('f', f)[0]
		lod.texture_index = self._unpack('b', f)[0]
		lod.translucent = self._unpack('b', f)[0]
		lod.cast_shadow = self._unpack('b', f)[0]
		lod.piece_count = self._unpack('I', f)[0]

		lod.piece_index_list = [ self._unpack('I', f)[0] for _ in range(lod.piece_count) ]

		return lod

	def _read_lod_group(self, f):
		piece = Piece()

		name_offset = self._unpack('I', f)[0]
		piece.name = self._get_string_from_table(name_offset)
		lod_count = self._unpack('I', f)[0]

		piece.lods = [ self._read_lod(f) for _ in range(lod_count) ]

		return piece

	def _read_pieces(self, f):
		debug_ftell = f.tell()

		lod_group_count = self._unpack('I', f)[0]

		# Not quite sure this is right...
		if lod_group_count == 0:
			return []

		pieces = [ self._read_lod_group(f) for _ in range(lod_group_count) ]

		# Unknown values!
		unk_1 = self._unpack('I', f)[0]
		unk_2 = self._unpack('I', f)[0]
		unk_3 = self._unpack('I', f)[0]

		# End unknown values

		if unk_1 < 2 and DISTRICT_187_TEST == False:
			raise Exception("Unsupported external mesh data found.")

		# Read the mesh data, and process the pieces
		pieces = self._read_mesh_data(pieces, f)

		# Read shadow geometry?
		#pieces = self._read_mesh_data(pieces, f)

		return pieces

	def _read_node(self, f):
		node = Node()
		name_offset = self._unpack('I', f)[0]
		node.name = self._get_string_from_table(name_offset)

		if self.version != 42:
			node.index = self._unpack('H', f)[0]

		node.flags = self._unpack('b', f)[0]

		node.location = self._read_vector(f)
		node.rotation = self._read_quaternion(f)

		# Transform location/rotation into a bind matrix!
		mat_rot = node.rotation.to_matrix()
		mat_loc = Matrix.Translation(node.location)
		node.bind_matrix = mat_loc @ mat_rot.to_4x4()

		node.inverse_bind_matrix = node.bind_matrix.inverted()
		node.child_count = self._unpack('I', f)[0]
		return node

	def _read_child_model(self, f):
		child_model = ChildModel()
		child_model.name = self._read_string(f)
		return child_model

	def _read_keyframe(self, f):
		keyframe = Animation.Keyframe()
		keyframe.time = self._unpack('I', f)[0]
		string_offset = self._unpack('I', f)[0]
		keyframe.string = self._get_string_from_table(string_offset)
		return keyframe

	def _read_socket(self, f):
		socket = Socket()
		socket.node_index = self._unpack('I', f)[0]

		name_offset = self._unpack('I', f)[0]
		socket.name = self._get_string_from_table(name_offset)

		socket.rotation = self._read_quaternion(f)
		socket.location = self._read_vector(f)

		# Only one float!
		socket.scale = self._unpack('f', f)[0]

		return socket

	def _read_anim_binding(self, f):
		anim_binding = AnimBinding()

		anim_binding.extents = self._read_vector(f)

		anim_binding.radius = self._unpack('f', f)[0]

		name_offset = self._unpack('I', f)[0]
		anim_binding.name = self._get_string_from_table(name_offset)

		anim_binding.interpolation_time = self._unpack('I', f)[0]

		anim_binding.animation_header_index = self._unpack('I', f)[0]
		anim_binding.data_position = self._unpack('I', f)[0]
		anim_binding.is_compressed = self._unpack('I', f)[0]

		fin = True

		return anim_binding

	def _read_anim_info(self, f):
		anim_info = self.AnimInfo()

		anim_info.binding = self._read_anim_binding(f)

		anim_info.animation.extents = anim_info.binding.extents
		anim_info.animation.interpolation_time = anim_info.binding.interpolation_time
		anim_info.animation.name = anim_info.binding.name
		anim_info.animation.keyframe_count = self._unpack('I', f)[0]
		anim_info.animation.keyframes = [self._read_keyframe(f) for _ in range(anim_info.animation.keyframe_count)]

		return anim_info

	def _read_weight_set(self, f):
		weight_set = WeightSet()
		name_offset = self._unpack('I', f)[0]
		weight_set.name = self._get_string_from_table(name_offset)

		unk_1 = self._unpack('I', f)[0]
		weight_set.node_weights = [unpack('f', f)[0] for _ in range(self.node_count)]

		return weight_set

	def _read_physics_shape(self, f):
		shape = self.PhysicsShape()

		shape.index = self._unpack('b', f)[0]
		shape.offset = self._read_vector(f)
		shape.orientation = self._read_quaternion(f)
		shape.cor = self._unpack('f', f)[0]
		shape.friction = self._unpack('f', f)[0]
		shape.collision_group = self._unpack('I', f)[0]

		# read havok shapes, TODO: clean up
		shape_count = 1
		havok_shape = None
		while shape_count > 0:
			type_id = self._unpack('I', f)[0]

			if type_id == 0x1:
				raise Exception("Unknown physics type_id 0x1 found.")
			elif type_id == 0x2: # box
				havok_shape = self.PhysicsShape.HavokShape()
				shape.havok_shapes.append(havok_shape)
				havok_shape.mass, havok_shape.density, havok_shape.radius = self._unpack('3f', f)
				_ = self._read_vector(f)
			elif type_id == 0x3: # sphere
				havok_shape = self.PhysicsShape.HavokShape()
				shape.havok_shapes.append(havok_shape)
				havok_shape.mass, havok_shape.density, havok_shape.radius = self._unpack('3f', f)
			elif type_id == 0x4:
				raise Exception("Unknown physics type_id 0x4 found.")
			elif type_id == 0x5: # meta shape - shape offset
				havok_shape.offset = self._read_vector(f)
				havok_shape.orientation = self._read_quaternion(f)
			elif type_id == 0x6: # meta shape - shape count
				shape_count += self._unpack('i', f)[0] * 2 - 1
			elif type_id == 0x7: # capsule
				havok_shape = self.PhysicsShape.HavokShape()
				shape.havok_shapes.append(havok_shape)
				havok_shape.mass, havok_shape.density, havok_shape.radius = self._unpack('3f', f)
				_ = self._read_vector(f)
				_ = self._read_vector(f)

			shape_count -= 1

		#print(shape.havok_shapes)

		return shape

	def _read_physics_constraint(self, f):
		constraint = self.PhysicsConstraint()

		constraint.type = self._unpack('I', f)[0]
		constraint.nodes = self._unpack('2I', f)

		data_length = 24

		if constraint.type == 3:
			data_length = 18

		constraint.data = [ self._unpack('f', f)[0] for _ in range(data_length) ]

		constraint.friction = self._unpack('I', f)[0]

		if constraint.type == 3:
			constraint.unk_2 = self._unpack('I', f)[0]
			constraint.unk_3 = self._unpack('I', f)[0]

		return constraint

	def _read_physics_node_weight(self, f):
		#node_set = PhysicsNodeWeights()
		physics = self._unpack('b', f)[0]
		velocity_gain = self._unpack('f', f)[0]
		hiearchy_gain = self._unpack('f', f)[0]
		return [] # node_set


	def _read_physics_weights(self, shape_count, f):
		#weight_set = PhysicsWeightSet()

		name_offset = self._unpack('I', f)[0]
		name = self._get_string_from_table(name_offset)

		node_weights = [ self._read_physics_node_weight(f) for _ in range(shape_count) ]

		return [] # weight_set

	def _read_physics(self, f):
		physics = self.Physics()
		physics.vis_node_index = self._unpack('I', f)[0]
		physics.vis_radius = self._unpack('f', f)[0]
		physics.shape_count = self._unpack('I', f)[0]

		# TODO: Read each physics-based count in the order they're placed in the header

		physics.shapes = [ self._read_physics_shape(f) for _ in range(physics.shape_count) ]

		physics.constraint_count = self._unpack('I', f)[0]
		physics.contraints = [ self._read_physics_constraint(f) for _ in range(physics.constraint_count) ]

		physics.weight_set_count = self._unpack('I', f)[0]
		physics.weight_sets = [ self._read_physics_weights(physics.shape_count, f) for _ in range(physics.weight_set_count)]

		return physics

	def _read_animation_schema(self, f):
		# Basically a map for how we'll read a particular animation

		track_1_size = self._unpack('H', f)[0]
		track_2_size = self._unpack('H', f)[0]

		compression_schema = [ self._unpack('2h', f) for _ in range(self.node_count) ]

		return compression_schema
	# End Def

	def from_file(self, path):
		model = Model()
		model.name = os.path.splitext(os.path.basename(path))[0]
		with open(path, 'rb') as f:

			file_format = self._read_fixed_string(4, f)

			# Are we big-endian?
			if file_format == "LDOM":
				print("!! Big-endian Model00p loaded. Haven't tested this yet, may be bugs!!!")
				self.is_little_endian = False
			# No, then make sure we're little endian
			elif file_format != "MODL":
				raise Exception('Unsupported File Format! Only Model00p files are supported.')
			# End If

			self.version = self._unpack('I', f)[0]

			# FEAR, Condemned, FEAR 2
			if self.version not in [33, 34, 42]:
				raise Exception('Unsupported File Version! Importer currently only supports v33/v34.')
			# End If

			model.version = self.version

			keyframe_count = self._unpack('I', f)[0]

			animation_count = self._unpack('I', f)[0]

			self.node_count = self._unpack('I', f)[0]
			piece_count = self._unpack('I', f)[0]
			child_model_count = self._unpack('I', f)[0]
			self.lod_count = self._unpack('I', f)[0]
			socket_count = self._unpack('I', f)[0]

			if self.version != 42:
				animation_weight_count = self._unpack('I', f)[0]
				animation_schema_count = self._unpack('I', f)[0]

			string_data_length = self._unpack('I', f)[0]
			physics_weight_count = self._unpack('I', f)[0]
			physics_shape_count = self._unpack('I', f)[0]
			unk_12 = self._unpack('I', f)[0] # ??
			unk_13 = self._unpack('I', f)[0] # ??
			# Physics Constraints
			stiff_sprint_constraint_count = self._unpack('I', f)[0]
			hinge_constraint_count = self._unpack('I', f)[0]
			limited_hinge_constraint_count = self._unpack('I', f)[0]
			ragdoll_constraint_count = self._unpack('I', f)[0]
			wheel_constraint_count = self._unpack('I', f)[0]
			prismatic_constraint_count = self._unpack('I', f)[0]

			# End
			if self.version != 42:
				animation_data_length = self._unpack('I', f)[0]

			self.string_table = self._read_fixed_string(string_data_length, f)

			#
			# Nodes
			#
			model.nodes = [self._read_node(f) for _ in range(self.node_count)]
			build_undirected_tree(model.nodes)

			if self.version == 42:
				return model

			#
			# Animations
			#
			unknown = self._unpack('f', f)[0]

			# What is it? We'll find out...one day...
			if unknown != 0:
				print("Unknown animation value is not 0! It's %d" % unknown)

			# Dictionary per animation, Location/Rotation
			animation_schemas = []

			for _ in range(animation_schema_count):
				animation_schemas.append(self._read_animation_schema(f))

			# Okay save the current position, and read ahead to the keyframe data
			animation_position = f.tell()

			# Skip ahead to keyframes!
			f.seek(animation_data_length , 1)

			anim_infos = [self._read_anim_info(f) for _ in range(animation_count)]

			weight_set_count = self._unpack('I', f)[0]
			model.weight_sets = [self._read_weight_set(f) for _ in range(weight_set_count)]

			# Only read animations we want toooo!
			# This is disabled for test use!
			if self._read_animations:
				animation_binding_position = f.tell()
				f.seek(animation_position, 0)

				#########################################################################
				# Animation Pass

				def decompress_vec(compressed_vec):
					for i in range(len(compressed_vec)):
						if compressed_vec[i] != 0:
							compressed_vec[i] /= 64.0

					return Vector( compressed_vec )

				# Not really it, but a starting point!
				def decompres_quat(compresed_quat):
					for i in range(len(compresed_quat)):
						if compresed_quat[i] != 0:
							compresed_quat[i] /= 32768.0

					return Quaternion( compresed_quat )

				# Should match up with animation count...
				for anim_info in anim_infos:
					section = animation_schemas[anim_info.binding.animation_header_index]

					base_transform = []

					# read the first frame of every node for the base transforms?
					for node_index in range(self.node_count):
						temp_transform = Animation.Keyframe.Transform()

						if section[node_index][0] < -1:
							if anim_info.binding.is_compressed: # 0 = not compressed, 1 = both compressed, 2 = only rotation compressed?
								temp_transform.location = decompress_vec(self._read_short_vector(f))
							else:
								temp_transform.location = self._read_vector(f)

						if section[node_index][1] < -1:
							temp_transform.rotation = decompres_quat(self._read_short_quaternion(f))

						base_transform.append(temp_transform)

					for keyframe_index in range(anim_info.animation.keyframe_count):
						for node_index in range(self.node_count):

							# Make sure we have space here...
							try:
								anim_info.animation.node_keyframe_transforms[node_index]
							except:
								anim_info.animation.node_keyframe_transforms.append([])

							transform = Animation.Keyframe.Transform()
							transform.location = base_transform[node_index].location
							transform.rotation = base_transform[node_index].rotation

							if (section[node_index][0] & 0x8000) == 0:
								if anim_info.binding.is_compressed:
									transform.location = decompress_vec(self._read_short_vector(f))
								else:
									transform.location = self._read_vector(f)

							if (section[node_index][1] & 0x8000) == 0:
								transform.rotation = decompres_quat(self._read_short_quaternion(f))

							# Insert the transform
							anim_info.animation.node_keyframe_transforms[node_index].append(transform)

						# End For (Node)
					# End For (Keyframe)

					model.animations.append(anim_info.animation)

				f.seek(animation_binding_position)


				# End Pass
				#########################################################################
			# End If

			#
			# Sockets
			#
			socket_count = self._unpack('I', f)[0]
			model.sockets = [self._read_socket(f) for _ in range(socket_count)]

			'''
			District 187 exclusive data:
			'''
			if DISTRICT_187_TEST:
				def _read_d187_unknown_1(f):
					count = self._unpack('I', f)[0]
					return [self._unpack('5I', f) for _ in range(count)]

				anim_unk = [_read_d187_unknown_1(f) for _ in range(animation_count)]

				def _read_animation_transition(f):
					return self._unpack('4I', f)

				anim_transition_count = self._unpack('I', f)[0]
				anim_transitions = [_read_animation_transition(f) for _ in range(anim_transition_count)]
			'''
			exclusive end
			'''

			#
			# Child Models
			#
			child_model_count = self._unpack('I', f)[0]

			# In v34 they reduced the count by 1. (Before child model count use to include itself!)
			if self.version == 34:
				child_model_count += 1

			model.child_models = [self._read_child_model(f) for _ in range(child_model_count - 1)]

			debug_ftell = f.tell()

			# Small flag determines if we excluded geometry on compile!
			has_geometry = self._unpack('b', f)[0]

			# No geomtry? Then let's exit!
			if has_geometry == 0:
				return model

			#
			# Physics
			#
			model.physics = self._read_physics(f)

			#
			# Pieces
			#
			model.pieces = self._read_pieces(f)

			return model