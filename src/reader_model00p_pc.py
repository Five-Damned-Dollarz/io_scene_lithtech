import os
from .abc import *
from .io import unpack
from mathutils import Vector, Matrix, Quaternion

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
		def __init__(self, data_size):
			self.vertex = Vector()
			self.normal = Vector()
			self.uvs = Vector()
			self.unk_1 = Vector()
			self.unk_2 = Vector()
			self.weight_info = []
			self.node_indexes = []
			self.colour = []

			# FEAR uses 64, which is mesh data WITHOUT colour info
			# Condemned uses 68, which includes colour info.
			self.data_size = data_size

			if data_size not in [64, 68]:
				print("WARNING: Non-standard MeshData size. Size is %d" % data_size)

		def read(self, reader, f):
			self.vertex = reader._read_vector(f)
			self.normal = reader._read_vector(f)
			self.uvs.xy = reader._unpack('2f', f)
			self.unk_1 = reader._read_vector(f)
			self.unk_2 = reader._read_vector(f)

			if self.data_size == 68:
				self.colour = reader._unpack('4B', f)

			self.weight_info = reader._unpack('3B', f)
			padding = reader._unpack('B', f)[0]
			self.node_indexes = reader._unpack('3B', f)
			padding = reader._unpack('B', f)[0]

			# Reverse the weight info, I'm not sure why it's flipped...
			self.weight_info = tuple(reversed(self.weight_info))

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
			self.unk_4 = 0
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
			self.unk_4 = reader._unpack('I', f)[0]
			self.influence_node_indexes = [ reader._unpack('b', f)[0] for _ in range(self.influence_count) ]

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
		def __init__(self):
			self.index = -1
			self.offset = -1
			self.orientation = Vector()
			self.cor = -1
			self.friction = -1
			self.collision_group = -1
			self.node_index = -1
			self.mass = -1
			self.density = -1
			self.radius = -1

			# Capsule specific
			# Since sphere doesn't have orientation data, this works?
			self.unk_1 = -1
			self.length_pt1 = -1
			self.unk_2 = -1
			self.unk_2 = -1
			self.length_pt1 = -1
			self.unk_2 = -1

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

		texture_count = self._unpack('I', f)[0]

		data_length = self._unpack('I', f)[0]
		index_list_length = self._unpack('I', f)[0]

		print("Mesh Data Triangle Count: ", data_length / 64)

		# We need to read the MeshInfo data that's located AFTER the actual mesh data
		# So skip the mesh data for now...

		mesh_data_position = f.tell()

		f.seek(data_length, 1)
		f.seek(index_list_length, 1)

		# Unknown after mesh data
		# These seem to be the same, so asserts are here for debug.
		unk_1 = self._unpack('I', f)[0]
		assert(unk_1 == 1)

		byte_list_count = self._unpack('I', f)[0]

		# Not sure what this is, but it I can safely ignore it for now.
		unk_byte_list = [ self._unpack('B', f)[0] for _ in range(byte_list_count) ]

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
				data = self.MeshData(info.mesh_data_size)
				mesh_data_list.append( data.read(self, f) )

		# END NEW

		print("Data List Length -> ", len(mesh_data_list))

		index_list = [ self._unpack('H', f)[0] for _ in range( int(index_list_length / 2) ) ]

		debug_ftell = f.tell()

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

		unk_4 = self._unpack('I', f)[0]
		unk_5 = self._unpack('I', f)[0]

		# End unknown values

		# Read the mesh data, and process the pieces
		pieces = self._read_mesh_data(pieces, f)

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

		# read params
		type_id = self._unpack('I', f)[0] # FIXME: if Blender upgrades to Python 3.8 just do: `while (type_id := self._unpack('I', f)[0]) <= 7:`
		while type_id <= 7:
			if type_id == 0x2:
				_ = self._unpack('3f', f)
				_ = self._read_vector(f)
			elif type_id == 0x3:
				_ = self._unpack('3f', f)
			elif type_id == 0x5:
				_ = self._read_vector(f)
				_ = self._read_quaternion(f)
			elif type_id == 0x6:
				_ = self._unpack('i', f)
			elif type_id == 0x7:
				_ = self._unpack('3f', f)
				_ = self._read_vector(f)
				_ = self._read_vector(f)

			type_id = self._unpack('I', f)[0]

		f.seek(-4, 1)

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