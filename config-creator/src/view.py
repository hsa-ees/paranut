#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2023      Daniel Bortkevych  <daniel.bortkevych@hs-augsburg.de>
#                          Oleg Murashko      <oleg.murashko1@hs-augsburg.de>
#                          Haris Vojic        <haris.vojic@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    This file implements the frontend of the application
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
import json
import logging
import sys
import os
import re
import threading
import time

import gi
gi.require_version('Gtk', '3.0')
from gi.repository		import Gtk 			as gtk
from gi.repository		import Gdk 			as gdk
from gi.repository		import GLib			as glib

from os					import listdir, remove
from math				import log2, log
from os.path			import isfile, join

from report_zybo7010	import Zybo7010		as zybo7010 
from report_zybo7020	import Zybo7020		as zybo7020


#  Global variables
#  -----------------------------------------------------------------------------
success = gdk.Color(40*255, 167*255, 69*255)
error 	= gdk.Color(220*255, 53*255, 69*255)

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

file_handler 	= logging.FileHandler('logger/view.log', mode = 'w')
console_handler	= logging.StreamHandler(stream=sys.stdout)

format="%(levelname)s:%(name)10.10s:%(funcName)20.20s:%(lineno)4d : %(message)s"
formatter = logging.Formatter(format)

file_handler.setFormatter(formatter)
console_handler.setFormatter(formatter)

logger.addHandler(file_handler)
logger.addHandler(console_handler)

## View class is used to render and navigate in the UI (User Interface)
class View(gtk.Window):
	## Constructor initializes a View object
	## @var builder  A Gtk.Builder instance for fetching Gtk.objects from glade 
	#  files.
	def __init__(self):
		logger.debug("Initialization started") 
		logger.debug("Initializing attributes")
		#  @var controller      A Controller instance for data manipulation.
		self.controller			= None
		self.builder			= gtk.Builder()
		self.window				= None
		self.dialog				= None
		self.hardware_resource	= 17600
		self.configuration		= None
		self.json_path			= "json"
		self.json_file			= ".default.json"
		self.glade_path			= "glade"

		self.utilization_table	= zybo7010.utilization_table
		self.configuration_list = zybo7010.configuration_list

		self.provider           = None
		self.is_style_dark      = 0

		logger.debug("Initializing builder")
		self.set_builder(f'start_page.glade')
		self.set_css(f'gtk_theme/gtk.css')
		self.window			= self.builder.get_object("main_window")
		self.window.connect("delete-event", gtk.main_quit)

		logger.debug("Display GUI")
		self.show_window()
		logger.debug(f'Initialisation finished')

	def threaded(fn):
		def wrapper(*args, **kwargs):
			threading.Thread(target=fn, args=args, kwargs=kwargs).start()
		return wrapper

	## Sets the controller instance.
	# 
	#  This function also loads the configuration from a json file into an 
	#  object.
	# 
	#  @param self			The object pointer.
	#  @param controller 	The Controller object.
	def set_controller(self, controller):
		logger.debug("Setting controller")
		self.controller = controller

	## Sets builder instance for specified glade file.
	#  
	#  This function also opens the main window of the glade file, and connects
	#  the signals. (Widget to function). If the file doesn't exist the program
	#  will be terminated.
	#  
	#  @param self			The object pointer.
	#  @param glade_file	String of the path to the glade file.
	def set_builder(self, glade_file: str):
		file = f'{self.glade_path}/{glade_file}'
		logger.debug(f'Reading glade file: {file}')
		if isfile(f'{file}'):
			self.builder.add_from_file(file)
			logger.debug("Connectiong signals")
			self.builder.connect_signals(self)
		else:
			logger.error(f'Glade file: "{file}" not found')
			sys.exit()

	## Sets the CSS style for the GUI.
	#
	#  @param css_file		String of the path to the CSS file. 
	def set_css(self, css_file: str):
		file = f'{self.glade_path}/{css_file}'
		logger.debug(f'Reading css file: {file}')
		if isfile(file):
			self.provider = gtk.CssProvider()
			self.provider.load_from_path(file)
			gtk.StyleContext.add_provider_for_screen(gdk.Screen.get_default(), 
				self.provider,
				gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)
		else:
			logger.error(f'Glade file: "{css_file}" not found')
			sys.exit()

	## Sets the position and size of the old window instance to the new window
	#  instance.
	#
	#  @param self			The object pointer.
	def show_window(self):
		logger.debug("Saving position and size form previous window")
		# @var variable
		position	= self.window.get_position()	
		size		= self.window.get_size()

		logger.debug("Destorying previous window")
		self.window.hide()
		self.window.unrealize()

		logger.debug("Fetching text window")
		self.window			= self.builder.get_object("main_window")
		self.window.connect("delete-event", gtk.main_quit)

		logger.debug("Loading position and size form previous window")
		self.window.move(position.root_x, position.root_y)
		self.window.resize(size.width, size.height)

		self.window.show()
		logger.debug("***************************")

	## Display error message in the top banner.
	#
	#  @param self			The object pointer.
	#  @param message       String to be displayed in banner
	def show_error(self, message: str):
		logger.debug("Fetching banner widget")
		banner 			= self.builder.get_object("banner")
		message_widget 	= self.builder.get_object("message_widget")

		logger.debug("Setting text in banner")
		message_widget.set_markup(f"<span foreground='#fff'>{message}</span>")

		logger.debug("Modifying banner style")
		banner.modify_bg(gtk.StateFlags.NORMAL, error)

		logger.debug(f'Displaying error message: "{message}"')
		#banner.show()
		banner.popup()
		self.hide_banner()

	## Display success message in the top banner.
	#
	#  @param self			The object pointer.
	#  @param message 		Message displayed in banner.
	def show_success(self, message: str):
		logger.debug("Fetching banner widget")
		banner 			= self.builder.get_object("banner")
		message_widget 	= self.builder.get_object("message_widget")

		logger.debug("Setting text in banner")
		message_widget.set_markup(f"<span foreground='#fff'>{message}</span>")

		logger.debug("Modifying banner style")
		banner.modify_bg(gtk.StateFlags.NORMAL, success)

		logger.debug(f'Displaying success message: "{message}"')
		#banner.show()
		banner.popup()
		self.hide_banner()
	
	## Hide banner in the top.
	#
	#  @param self			The object pointer.
	@threaded
	def hide_banner(self):
		time.sleep(3)
		logger.debug("Fetching banner widget")
		banner = self.builder.get_object("banner")
		logger.debug("Hiding banner widget")
		glib.idle_add(banner.popdown)
		#banner.popdown()
			
	## Callback function for a gtk.Widget. Reopens "auto_page.glade"
	def on_button_select_clicked(self, Widget: gtk.Widget):
		list_box = self.builder.get_object("list_box")
		row			= list_box.get_selected_row()
		if row:
			item 		= row.get_child()
			self.json_file = item.get_text()
			self.controller.reload_object()
			self.on_button_final_page_from_auto_page_clicked(Widget)
			#self.hide_banner()
		else:
			message = "Error: Please select file"
			self.show_error(message)

		
	## Input proofing callback function, that checks wether the Widget value is 
	#  log base 2.
	#
	#  @param self			The object pointer.
	#  @param Widget		Gtk widget that calls this callback function.
	def check_ld(self, Widget: gtk.Widget, *args):
		logger.debug("Checking ld")
		message = None
		try:
			value = int(Widget.get_text())
			value = int(log(value, 2))
			value1 = 2**value
			value2 = 2**(value+1)

			if value1 != int(Widget.get_text()):

				message = f'Error: The value = {Widget.get_text()} '
				message = message + 'is not supported.\n'
				message = message + 'The value must be a power of 2, '
				message = message + f'e.g. {value1} or {value2}.'

				Widget.grab_focus()
				self.show_error(message)
		
		except ValueError:
			message = f'Error: The value must be a power of 2!'

			Widget.grab_focus()
			self.show_error(message)

	## Input proofing callback functions,that calculates and sets the amount of 
	#  cores.
	#
	#  @param self			The object pointer.
	#  @param *args			Gtk widget that calls this callback function.
	def check_cores(self, Widget: gtk.Widget, *args):
		logger.debug("Checking cores")
		try:
			logger.debug("Fetching gtk objects")
			cores_cap1 		= self.builder.get_object("cpu_cap1_cores")
			cores_cap2 		= self.builder.get_object("cpu_cap2_cores")
			cores_cap3 		= self.builder.get_object("cpu_cap3_cores")
			cores_ld		= self.builder.get_object("cpu_cores_ld")

			logger.debug("Converting str to int")
			number_of_cores			= int(cores_ld.get_text())
			number_of_cap1_cores 	= int(cores_cap1.get_text())
			
			if number_of_cores == 0:
				cores_cap1.set_text("0")
				cores_cap2.set_text("0")
				cores_cap3.set_text("0")
			else:
				if (number_of_cores) <= number_of_cap1_cores:
					number_of_cap1_cores = number_of_cores - 1

					message = 'Warning: The maximum number of capability-1-'
					message = message + 'cores\n can not be greather ' 
					message = message + f'{number_of_cap1_cores}!'
					self.show_error(message)

				cores_cap1.set_text(str(number_of_cap1_cores))
				cores_cap2.set_text(str(number_of_cores-number_of_cap1_cores-1))
				cores_cap3.set_text("1")

		except ValueError:
			message = f'Error: The value must be a number!'

			Widget.grab_focus()
			self.show_error(message)

		except:
			logger.error("error")	

	## Input proofing callback function, that checks wether the Widget value 
	#  contains digits. All non digits are removed. The value is limited to 5
	#  digits.
	#
	#  @param self			The object pointer.
	#  @param Widget		Gtk widget that calls this callback function.
	def check_number(self, Widget, *args):
		logger.debug("Checking number")
		message = None
		value = Widget.get_text().strip()
		try:
			value = str(int(value))

		except:
			Widget.grab_focus()
			if value != "":
				message = f'Error: The value must be a number!'
				self.show_error(message)
				value = ''.join(char for char in value if char.isdigit())
		Widget.set_text(value)
	

	## Input proofing callback function, that checks wether the Widget value 
	#  does not contain special characters.
	#
	#  @param self			The object pointer.
	#  @param Widget		Gtk widget that calls this callback function.
	def check_path(self, Widget: gtk.Widget):
		value = Widget.get_text()
		Widget.set_text(value.strip('!@#$%^&*()+[]\{\}|\<>'))


	## Input proofing callback function, that checks wether the Widget value
	#  is in range [-1, 15].
	#
	#  @param self			The object pointer.
	#  @param Widget		Gtk widget that calls this callback function.
	def check_arbiter_method(self, Widget: gtk.Widget, *args):
		value = Widget.get_text().strip()
		value = ''.join(char for char in value if char.isdigit() or char == '-')
		if len(value) > 1:
			value = value[0] + value[1:].replace('-', '')
		try:
			if int(value) in range(-1, 16):
				value = str(int(value))
			else:
				raise
		except:
			Widget.grab_focus()
			message = f'Error: Number must be in range [-1, 15]'
			self.show_error(message)
		Widget.set_text(value)

	def check_integer(self, Widget, *args):
		logger.debug("Checking integer")
		value = Widget.get_text().strip()
		try:
				value = str(int(value))
		except:
			Widget.grab_focus()
			if value != "" and value != "-":
				message = f'Error: The value must be a number!'
				self.show_error(message)
				value = ''.join(char for char in value if char.isdigit() or char == '-')
				if len(value) > 1:
					value = value[0] + value[1:].replace('-', '')
		Widget.set_text(value)

	#
	#  @param self			The object pointer.
	#  @param Widget		Gtk widget that calls this callback function.
	def on_hardware_changed(self, Widget: gtk.Widget):
		logger.debug("Hardware changed")
		self.hardware_resource 	= int(Widget.get_active_id())
		
		if self.hardware_resource == 17600:
			self.utilization_table 	= zybo7010.utilization_table
			self.configuration_list = zybo7010.configuration_list
		elif self.hardware_resource == 53200:
			self.utilization_table 	= zybo7020.utilization_table
			self.configuration_list = zybo7020.configuration_list

#  Buttons/Page navigation
#  -----------------------------------------------------------------------------
	## Opens edit_page glade file.
	#
	#  @param self			The object pointer.
	#  @param Widget		Gtk widget that calls this callback function.
	def on_button_edit_page_clicked(self, Widget: gtk.Widget):
		file = f'edit_page.glade'
		logger.debug(f'Loading builder form: "{file}"')
		self.controller._load_configuration()
		self.set_builder(file)
		logger.debug(f'Loading configurtation from objects to gtk_objects')
		self.load_configuration()
		self.check_cores(Widget)
		logger.debug(f'Loading progress bar')
		self.load_progress_bar()
		logger.debug(f'Showing edit_page')
		self.show_window()

	## Opens edit_page glade file from final_page.
	#
	#  @param self			The object pointer.
	#  @param Widget		Gtk widget that calls this callback function.
	def on_button_edit_page_from_final_clicked(self, Widget: gtk.Widget):
		file = f'edit_page.glade'
		self.set_builder(file)
		logger.debug(f'Loading configurtation from objects to gtk_objects')
		self.load_configuration()
		self.check_cores(Widget)
		logger.debug(f'Loading progress bar')
		self.load_progress_bar()
		logger.debug(f'Showing edit_page')
		self.show_window()

	## Opens final_page glade file.
	#
	#  @param self			The object pointer.
	#  @param Widget		Gtk widget that calls this callback function.
	def on_button_final_page_from_auto_page_clicked(self, Widget: gtk.Widget):
		file = f'final_page.glade'
		logger.debug(f'Loading builder form: "{file}"')
		self.set_builder(file)
		logger.debug(f'Loading hardware')
		self.load_hardware()
		logger.debug(f'Loading configuration form objects to gtk_objects')
		self.load_configuration()
		self.check_cores(Widget)
		logger.debug(f'Showing final_page')
		self.show_window()

	## Opens final_page glade file.
	#
	#  @param self			The object pointer.
	#  @param Widget		Gtk widget that calls this callback function.
	def on_button_final_page_clicked(self, Widget: gtk.Widget):
		logger.debug(f'Storing data form gtk_object to object')
		self.store_data()
		file = f'final_page.glade'
		logger.debug(f'Loading builder form: "{file}"')
		self.set_builder(file)
		logger.debug(f'Loading hardware')
		self.load_hardware()
		logger.debug(f'Loading configuration form objects to gtk_objects')
		self.load_configuration()
		self.check_cores(Widget)
		logger.debug(f'Showing final_page')
		self.show_window()

	## Opens start_page glade file.
	#
	#  @param self			The object pointer.
	#  @param Widget		Gtk widget that calls this callback function.
	def on_button_start_page_clicked(self, Widget: gtk.Widget):
		self.json_file = '.default.json'
		file = f'start_page.glade'
		logger.debug(f'Loading builder form: "{file}"')
		self.set_builder(file)
		# sets default value for CSS Switcher Combobox
		switcher = self.builder.get_object("css_switcher")
		switcher.set_active_id(str(self.is_style_dark))
		logger.debug(f'Loading hardware')
		self.load_hardware()
		logger.debug(f'Showing start_page')
		self.show_window()

	## Opens auto_page glade file.
	#
	#  @param self			The object pointer.
	#  @param Widget		Gtk widget that calls this callback function.
	def on_button_auto_page_clicked(self, Widget: gtk.Widget):
		file = f'auto_page.glade'
		logger.debug(f'Loading builder form: "{file}"')
		self.set_builder(file)
		logger.debug(f'Loading list')
		self.load_items()
		logger.debug(f'Showing auto_page')
		self.show_window()

	## Opens save_page glade file.
	#
	#  @param self			The object pointer.
	#  @param Widget		Gtk widget that calls this callback function.
	def on_button_file_chooser_dialog_clicked(self, Widget: gtk.Widget):
		file = f'save_page.glade'
		logger.debug(f'Loading builder form: "{file}"')
		self.window.set_sensitive(False)
		self.set_builder(file)

		logger.debug(f'Fetching "main_window" object')
		self.dialog			= self.builder.get_object("main_window")
		# FIXME: closing not working correctly
		self.dialog.set_action(gtk.FileChooserAction.SELECT_FOLDER)
		logger.debug(f'Connecting signals')
		self.dialog.connect("delete-event", self.on_button_cancel_file_chooser_dialog_clicked)
		logger.debug(f'Showing GUI')
		self.dialog.show()
		self.dialog.set_keep_above(True)
		
	## Closes file chooser dialog window.
	#
	#  @param self			The object pointer.
	#  @param Widget		Gtk widget that calls this callback function.
	def on_button_cancel_file_chooser_dialog_clicked(self, Widget, *args):
		self.dialog.hide()
		self.dialog.unrealize()	
		self.window.set_sensitive(True)

	## Saves "config.mk" file and closes file chooser window.
	#
	#  @param self			The object pointer.
	#  @param Widget		Gtk widget that calls this callback function.
	def on_button_save_file_chooser_dialog_clicked(self, Widget: gtk.Widget):
		filename 	= "config.mk"
		path		= self.dialog.get_current_folder()
		if path and self.controller: 
			message = None
			self.dialog.hide()
			self.dialog.unrealize()
			self.window.set_sensitive(True)
			if os.path.isfile(f'{path}/{filename}'):
				logger.debug(f'"{filename}" exists.')

				dialog = self.builder.get_object("dialog")
				area = self.builder.get_object("dialog_message")

				area_text = f'Configuration file "{filename}" already exists.\n'
				area_text = area_text + f'Do you want to overwrite it?'
				area.set_text(area_text)

				dialog.show_all()
				response = dialog.run()
				dialog.hide()

				if(response == gtk.ResponseType.OK):
					logger.debug(f'Overwrithing the file {filename}')
					self.controller.model.make_configuration(path)	
					message = f'Success: Configuration "{filename}" overwritten.'
					#self.show_success(message)

				elif(response == gtk.ResponseType.CANCEL):
					logger.debug(f'Abborting save process')
			
			else:
				logger.debug(f'Saving the file {filename}')
				message = f'Success: Configuration "{filename}" was saved.'
				self.controller.model.make_configuration(path)
				#self.show_success(message)
				
			self.on_button_final_page_clicked(Widget)
			if message:
				self.show_success(message)
		else:
			self.show_error(f'Error: Please select a directory.')

	## Opens notebook on general page.
	#
	#  @param self			The object pointer.
	#  @param Widget		Gtk widget that calls this callback function.
	def on_button_goto_general_clicked(self, Widget: gtk.Widget):
		self.on_button_edit_page_from_final_clicked(Widget)
		notebook = self.builder.get_object("notebook")
		notebook.set_current_page(0)
		
	## Opens notebook on exu page.
	#
	#  @param self			The object pointer.
	#  @param Widget		Gtk widget that calls this callback function.
	def on_button_goto_exu_clicked(self, Widget: gtk.Widget):
		self.on_button_edit_page_from_final_clicked(Widget)
		notebook = self.builder.get_object("notebook")
		notebook.set_current_page(1)

	## Opens notebook on memu page.
	#
	#  @param self			The object pointer.
	#  @param Widget		Gtk widget that calls this callback function.
	def on_button_goto_memu_clicked(self, Widget: gtk.Widget):
		self.on_button_edit_page_from_final_clicked(Widget)
		notebook = self.builder.get_object("notebook")
		notebook.set_current_page(2)

	## Opens notebook on mmu page.
	#
	#  @param self			The object pointer.
	#  @param Widget		Gtk widget that calls this callback function.
	def on_button_goto_mmu_clicked(self, Widget: gtk.Widget):
		self.on_button_edit_page_from_final_clicked(Widget)
		notebook = self.builder.get_object("notebook")
		notebook.set_current_page(3)

		
	## Opens notebook on ifu page.
	#
	#  @param self			The object pointer.
	#  @param Widget		Gtk widget that calls this callback function.
	def on_button_goto_ifu_clicked(self, Widget: gtk.Widget):
		self.on_button_edit_page_from_final_clicked(Widget)
		notebook = self.builder.get_object("notebook")
		notebook.set_current_page(4)

	## Opens notebook on lsu page.
	#
	#  @param self			The object pointer.
	#  @param Widget		Gtk widget that calls this callback function.
	def on_button_goto_lsu_clicked(self, Widget: gtk.Widget):
		self.on_button_edit_page_from_final_clicked(Widget)
		notebook = self.builder.get_object("notebook")
		notebook.set_current_page(5)

	## Changes UI CSS style
	#
	#  @param self			The object pointer.
	#  @param Widget		Gtk widget that calls this callback function.
	def on_css_switcher_changed(self, Widget: gtk.Widget):
		switcher = self.builder.get_object("css_switcher")
		self.is_style_dark = int(switcher.get_active_id())
		logger.debug(f'CSS Thema is {self.is_style_dark}')
		if self.is_style_dark == 1:
			logger.debug("CSS Thema is Dark")
			self.set_css("gtk_theme/gtk-dark.css")
		elif self.is_style_dark == 0:
			logger.debug("CSS Thema is Light")
			self.set_css("gtk_theme/gtk.css")
		
#  Controller calls
#  -----------------------------------------------------------------------------
	## Deletes selected preset from list and JSON file from ./json directory. 
	#
	#  @param self			The object pointer.
	#  @param Widget		Gtk widget that calls this callback function.
	def on_button_delete_preset_clicked(self, Widget: gtk.Widget):
		list_box 	= self.builder.get_object("list_box")
		row 		= list_box.get_selected_row()
		if row:
			preset_file	= row.get_child().get_text()

			response = None
			logger.debug("Preset already exists")
			dialog = self.builder.get_object("dialog")

			area = self.builder.get_object("dialog_message")
			message = f'Are you sure you want to delete "{preset_file}"?'
			area.set_text(message)

			dialog.show_all()
			response = dialog.run()
			dialog.hide()

			if(response == gtk.ResponseType.OK):
				logger.debug(f'Deleting: "./json/{preset_file}.json"')
				remove(f'./json/{preset_file}.json')
				row.destroy()
				message = f'Success: Preset file "{preset_file}" was deleted!'
				self.show_success(message)

			elif(response == gtk.ResponseType.CANCEL):
				logger.debug(f'Abborting delete process')
			
		else:
			message = "Error: Please select file"
			self.show_error(message)
	## Saves preset as JSON in ./json. 
	#
	#  @param self			The object pointer.
	#  @param Widget		Gtk widget that calls this callback function.
	def on_button_save_configuration(self, Widget: gtk.Widget, *arg):
		entry = self.builder.get_object("filename")
		filename = entry.get_text().strip()
		if filename == "":
			entry.grab_focus()
			message = f'Error: No name given, can not save.'
			self.show_error(message)

		elif self.controller:
			logger.debug(os.getcwd())
			logger.debug(f'./json/{filename}.json')
			response = None
			if(os.path.isfile(f'./json/{filename}.json')):
				logger.debug("Preset already exists")
				dialog = self.builder.get_object("dialog")

				area = self.builder.get_object("dialog_message")
				message = f'Preset "{filename}" already exists.\n'
				message = message + f'Do you want to overwrite it?'
				area.set_text(message)

				dialog.show_all()
				response = dialog.run()
				dialog.hide()

			if(response == gtk.ResponseType.OK):
				logger.debug(f'Overwrithing the file {filename}')
				self.controller.save_configuration(filename)
				message = f'Success: Preset "{filename}" overwritten.'
				self.show_success(message)

			elif(response == gtk.ResponseType.CANCEL):
				logger.debug(f'Abborting save process')
			
			else:
				logger.debug(f'Saving the file {filename}')
				self.controller.save_configuration(filename)
				message = f'Success: Preset "{filename}" was saved.'
				self.show_success(message)
	
	## Calculates hardware usage and displays progress bar Widget.
	#
	#  @param self			The object pointer.
	#  @param *args			Gtk widget that calls this callback function.
	def load_progress_bar(self, *args):
		logger.debug("Loading progress bar")
		configuration = list()
		for id_widget in self.configuration_list:
			data = ""
			gtk_object = self.builder.get_object(id_widget)
			try:
				if gtk_object.__class__.__name__ == "Entry":
					if "ld" in id_widget:
						data = gtk_object.get_text()
						data = int(data)
						data = int(log2(data))
						data = str(data)
					else:
						data = gtk_object.get_text()
				elif id_widget == "perfcounter_bits":
					data = int(gtk_object.get_value())
					data = str(data)
				elif gtk_object.__class__.__name__ == "CheckButton":
					if(gtk_object.get_active()):
						data = "1"
					else:
						data = "0"
				elif gtk_object.__class__.__name__ == "ComboBoxText":
					data = gtk_object.get_active_id()
			except:
				data = id_widget
			configuration.append(data)


		try:
			self.configuration = ''.join(configuration)
			progress = self.utilization_table.get(self.configuration)
			if progress == None:
				progress = 0

			progress_bar = self.builder.get_object('progress_bar')

			fraction = int(progress) / self.hardware_resource
			if 0 < fraction <= 1:
				progress_bar.modify_fg(gtk.StateFlags.NORMAL, success)

				message = f'{int(fraction * 100)} % ('
				message = message + f'{progress}/{self.hardware_resource}) LUTs'

				progress_bar.set_text(message)
			elif 0 == fraction:
				progress_bar.modify_fg(gtk.StateFlags.NORMAL, error)
				message = f'Warning: Hardware resources could not be estimated!'
				progress_bar.set_text(message)
				
			else:
				progress_bar.modify_fg(gtk.StateFlags.NORMAL, error)
				
				message = f'Warning: Insufficient hardware resources! '
				message = message + f'{int(fraction * 100)} % ('
				message = message + f'{progress}/{self.hardware_resource}) LUTs'
				progress_bar.set_text(message)
			
			progress_bar.set_fraction(fraction)
		except:
			logger.error("Not enugh data to calculate")

	## Sets "hardware" Widget to self.hardware_resource attribute.
	#
	#  @param self			The object pointer.
	def load_hardware(self):
		logger.debug("Fetching hardware widget")
		hardware = self.builder.get_object("hardware")
		logger.debug("Setting active id from self.hardware_resource")
		hardware.set_active_id(str(self.hardware_resource))

	## Displays all JSON presets from ./json directory in Gtk.ListBox Widget.
	#
	#  @param self			The object pointer.
	def load_items(self):
		list_box = self.builder.get_object("list_box")
		files = [f for f in listdir(self.json_path) if isfile(join(self.json_path,f))]
		files.sort(key=str.lower)
		for file in files:
			if not file.startswith('.'):
				item = gtk.Label()
				item.set_text(file.removesuffix('.json'))
				item.set_halign(gtk.Align.START)
				item.set_size_request(-1, 30)
				row		= gtk.ListBoxRow()
				row.add(item)
				list_box.add(row)
		list_box.show_all()

	## Loads configuration from Model to View.
	#
	#  @param self			The object pointer.
	def load_configuration(self):
		if self.controller:
			self.controller.load_configuration()			

	## Stores data from View to Model.
	#
	#  @param self			The object pointer.
	def store_data(self, *args):
		if self.controller:
			self.controller.store_data()

if __name__ == '__main__':
	view = View()
