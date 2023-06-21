#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2022-2023 Daniel Bortkevych  <daniel.bortkevych@hs-augsburg.de>
#                          Oleg Murashko      <oleg.murashko1@hs-augsburg.de>
#                          Haris Vojic        <haris.vojic@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    The business logic of the application is implemented in this file
#
#  --------------------- LICENSE -----------------------------------------------
#  Redistribution and use in source and binary forms, with or without 
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice, 
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation 
#     and/or other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
#  SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND 
#  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#  -----------------------------------------------------------------------------

#  Imports
#  -----------------------------------------------------------------------------
import 	json
import 	os
from 	json 	import JSONEncoder
from	paranut import *

## Object encoder class object to JSON file.
class obj_encoder(JSONEncoder):
	## Converts an object to dict.
	#
	#	@return			A dict of the object.
	def default(self, o: object):
		assert isinstance(o, object), 'Invalid object'
		assert o, 'Empty object'

		return o.__dict__
	
## Model class provides functions to load and store JSON confgiurations.
class Model:
	## The Configuration class initializer.
	#  Automaticaly calls the initializer for the units contained in kwargs.
	#
	#  @param self			The object pointer.
	def __init__(self):
		self._read_configuration()
	
	## Protected function, used to initialize processor units.
	#
	#  @param self			The object pointer.
	#  @param json_path		String of the JSON file, to be loaded.
	def _read_configuration(self, json_path = "json/.default.json"):
		try:
			with open(json_path, "r") as file:
				obj_json = file.read()

			self.configuration = json.loads(obj_json)

			self.description = self.configuration.get('description')

			self.nut 	= self.configuration.get('_Model__nut')
			self.exu 	= self.configuration.get('_Model__exu')
			self.memu 	= self.configuration.get('_Model__memu')
			self.mmu 	= self.configuration.get('_Model__mmu') 
			self.ifu 	= self.configuration.get('_Model__ifu')
			self.lsu 	= self.configuration.get('_Model__lsu')

		except FileNotFoundError:
			print(f'File "{json_path}" does not exist.')
	
		except json.decoder.JSONDecodeError:
			print(f'File "{json_path}" corrupted.')
		
		except ValueError:
			print(f'File "{json_path}" has wrong data.')

#  Getter methods for the privte meambers.
#  -----------------------------------------------------------------------------
	@property
	def nut(self):
		return self.__nut

	@property
	def exu(self):
		return self.__exu

	@property
	def mmu(self):
		return self.__mmu

	@property
	def ifu(self):
		return self.__ifu

	@property
	def lsu(self):
		return self.__lsu

	@property
	def memu(self):
		return self.__memu

#  Setter methods of the private processor units.
#  -----------------------------------------------------------------------------
	@nut.setter
	def nut(self, value):
		if type(value) is dict:
			self.__nut = NUT(**value)
		else:
			raise ValueError(f'Invalid dict: {value}')

	@exu.setter
	def exu(self, value):
		if type(value) is dict:
			self.__exu = EXU(**value)
		else:
			raise ValueError(f'Invalid dict: {value}')

	@mmu.setter
	def mmu(self, value):
		if type(value) is dict:
			self.__mmu = MMU(**value)
		else:
			raise ValueError(f'Invalid dict: {value}')

	@ifu.setter
	def ifu(self, value):
		if type(value) is dict:
			self.__ifu = IFU(**value)
		else:
			raise ValueError(f'Invalid dict: {value}')

	@lsu.setter
	def lsu(self, value):
		if type(value) is dict:
			self.__lsu = LSU(**value)
		else:
			raise ValueError(f'Invalid dict: {value}')

	@memu.setter
	def memu(self, value):
		if type(value) is dict:
			self.__memu = MEMU(**value)
		else:
			raise ValueError(f'Invalid dict: {value}')

#  Load/ Store functions.
#  -----------------------------------------------------------------------------
	## Creates "config.mk" configuration.
	#
	#  @param self			The object pointer.
	#  @param path 			Path of the saving directory.
	def make_configuration(self, path = "."):
		with open(f'{path}/config.mk', "w") as file:
			file.write('\n'.join(self.description))

		for unit in self.get_units():
			if unit == "description":
				 continue
			attr = getattr(self, unit)
			attr._write_unit(path)

	## Exports configuration as JSON.
	#
	#  @param self			The object pointer.
	#  @param file_name		The name of the JSON file.
	def save_configuration(self, filename: str):
		if(".json" not in filename):
			filename = f'{filename}.json'

		with open(f'json/{filename}', "w") as file:
			file.write(json.dumps(self, indent=4, cls=obj_encoder))

	# FIXME: not a solution

	## Retrives all units in the configuration.
	#
	#  @param self			The object pointer.
	#  @return 				A list of units of the configuration.
	def get_units(self):
		units = ['nut', 'exu', 'memu', 'mmu', 'lsu', 'ifu']
		return units

if __name__ == '__main__':

	conf_obj = Model()
	conf_obj.make_configuration()

	print(conf_obj.exu.get_labels())
	print(conf_obj.mmu.get_labels())
	print(conf_obj.nut.get_labels())
	print(conf_obj.nut.get_values())
	print(conf_obj.get_units())
	conf_obj.save_configuration("new")
