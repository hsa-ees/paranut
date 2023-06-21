#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2023-2024 Daniel Bortkevych  <daniel.bortkevych@hs-augsburg.de>
#                          Oleg Murashko      <oleg.murashko1@hs-augsburg.de>
#                          Haris Vojic        <haris.vojic@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    Transfers requests and responses between View and Model
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

## @package Controller
import math
import logging
import sys

logger			= logging.getLogger(__name__)

file_handler	= logging.FileHandler('logger/controll.log', mode = 'w')
console_handler	= logging.StreamHandler(stream = sys.stdout)

format = "%(levelname)s:%(name)10.10s:%(funcName)20.20s:%(lineno)4d:%(message)s"
formatter		= logging.Formatter(format)

file_handler.setFormatter(formatter)
console_handler.setFormatter(formatter)

logger.setLevel(logging.DEBUG)
logger.addHandler(file_handler)
logger.addHandler(console_handler)

##	Controller class is responsible for the communication between the other 
#	modules	of the MVC-model.
class Controller:
	## Constructor which takes external model and view to operate with.
	#
	#  @param self The object pointer.
	#  @param model Model object
	#  @param view View object
	#
	#  @return An instance of the Controller class initialized with the 
	#  specified View and Model object.
	def __init__(self, model, view):
		logger.debug(f'Initialization started')
		logger.debug(f'Initializing model and view')
		
		self.model	= model;
		self.view	= view;

		logger.debug(f'Initialization finished')


		
	# Getter methods
	#---------------------------------------------------------------------------
	@property
	def model(self):
		return self.__model

	@property
	def view(self):
		return self.__view

	# Setter methods
	#---------------------------------------------------------------------------
	@model.setter
	def model(self, value):
		if value.__class__.__name__ == 'Model':
			self.__model = value
		else:
			raise ValueError(f'Invalid model: {value}')

	@view.setter
	def view(self, value):
		if value.__class__.__name__ == 'View':
			self.__view = value
		else:
			raise ValueError(f'Invalid view: {value}')

	# Load/Store methods
	#---------------------------------------------------------------------------

	## Reloads configuration data from json file and saves them into a model 
	#  object.
	#
	#  @param self			The object pointer.
	def reload_object(self):

		## @var file		The JSON file to be opened.
		file = f'{self.view.json_path}/{self.view.json_file}.json'
		logger.debug(f'Reloading file: {file}')
		self.model._read_configuration(file)
	
	## Saves configuraiton as JSON.
	#
	#  @param self			The object pointer.
	#  @param filename		String name of the file to be saved.
	def save_configuration(self, filename):
		logger.debug(f'Saving configuration under: {filename}')	
		self.model.save_configuration(filename)

	## Loads configuration from self.config to glade.
	#
	#  @param self			The object pointer.
	def load_configuration(self):
		logger.debug(f'Loading configuration from objects to gtk_objects')
		for unit in self.model.get_units():
			# NOTE: potential bug solution
			if unit == "description":
				continue
			module = getattr(self.model, unit)

			for setting in module.get_attributes():
				gtk_object = self.view.builder.get_object(setting)

				attribute = getattr(module, setting)
				try:
					if "ld" in setting:
						ld = int(attribute.get('value'))
						ld = str(2**ld)

						gtk_object.set_text(ld)

					elif "cap1" in setting:
						gtk_object.set_text(attribute.get('value'))
						self.view.check_cores()

					elif "mem_size" in setting:
						value = attribute.get('value')
						value = ''.join(ch for ch in value if ch.isdigit())
						
						gtk_object.set_text(value)
				
					elif "perfcounter_bits" in setting:
						gtk_object.set_value(int(attribute.get('value')))

					else:
						gtk_object.set_text(attribute.get('value'))
				except:
					try:
						gtk_object.set_active_id(attribute.get('value'))
					# TODO: checkbox and radiobuttons
					except:
						try:
							if attribute.get('value') == '1':
								gtk_object.set_active(True)
							else:
								gtk_object.set_active(False)
						except:
							logger.error(f'Failed to edit: {setting}')

	## This function stores the configuration values from a View object into a 
	#  Model object.
	#
	#  @param self			The object pointer.
	def store_data(self):
		logger.debug(f'Storing date from gtk_objects to objects')
		for unit in self.model.get_units():
			if unit == "description":
				continue
			module = getattr(self.model, unit)
			for setting in module.get_attributes():
				gtk_object = self.view.builder.get_object(setting)
				
				attribute = getattr(module, setting)
				try:
					if gtk_object.__class__.__name__ == "Entry":
						if setting == "mem_size":
							value = f'({gtk_object.get_text()} * MB)'
							attribute.update({"value": value}) 

						elif "ld" in setting:
							ld = gtk_object.get_text()
							ld = int(ld)
							ld = int(math.log2(ld))
							ld = str(ld)
							attribute.update({"value": ld})

						else:
							value = gtk_object.get_text()
							attribute.update({"value": value})
					elif setting == "perfcounter_bits":
						value = int(gtk_object.get_value())
						value = str(value)

						attribute.update({"value": value})

					elif gtk_object.__class__.__name__ == "CheckButton":
						if(gtk_object.get_active()):
							attribute.update({"value": "1"})
						else:
							attribute.update({"value": "0"})
					
					elif gtk_object.__class__.__name__ == "ComboBoxText":
						value = gtk_object.get_active_id()
						attribute.update({"value": value})
				except:
					logger.error("aloha")
					
	## Creates "config.mk" configuration file.
	#
	#
	#  @param self			The object pointer.
	#  @pram path           Path to the saving location of "config.mk"
	def make_configuration(self, path = ""):
		try:
			self.model.make_configuration(path = "")
		except ValueError as error:
			self.view.show_error(error)

	## Reads data into objects from a JSON file.
	#
	#  @param self			The object pointer.
	def _load_configuration(self):
		json_path = f'{self.view.json_path}/{self.view.json_file}'
		logger.debug(f'Loading json configuration file {json_path}')
		self.model._read_configuration(json_path)
