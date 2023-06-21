#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2022      Daniel Bortkevych  <daniel.bortkevych@hs-augsburg.de>
#                          Oleg Murashko      <oleg.murashko1@hs-augsburg.de>
#                          Haris Vojic        <haris.vojic@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    This file is needed to start the program.
#	 It binds Model, View, Controller
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

import logging.config
import sys
import os

try:
	os.chdir(os.path.dirname(sys.argv[0]))
	print(os.getcwd())
except:
  pass

try:
	import gi
	gi.require_version('Gtk', '3.0')
except:
  pass

try:
	from gi.repository import Gtk 			as gtk
except:
  logger.debug('GTK not available')
  sys.exit(1)

from model 		import Model
from view 		import View
from controller import Controller

logger			= logging.getLogger(__name__)

file_handler	= logging.FileHandler('logger/app.log', mode = 'w')
console_handler = logging.StreamHandler(stream = sys.stdout)

format = "%(levelname)s:%(name)10.10s:%(funcName)20.20s:%(lineno)4d:%(message)s"
formatter 		= logging.Formatter(format)

file_handler.setFormatter(formatter)
console_handler.setFormatter(formatter)

logger.setLevel(logging.DEBUG)
logger.addHandler(file_handler)
logger.addHandler(console_handler)

class App():
	def __init__(self):
		super().__init__()
		logger.debug(f'Initialization started')

		# create a model
		logger.debug(f'Initializating Model')
		model = Model()

		# create a view 
		logger.debug(f'Initializating View')
		view = View()
		
		# create a controller
		logger.debug(f'Initializating Controller')
		controller = Controller(model, view)

		# set the controller to view
		logger.debug(f'Initializating Controller in View')
		view.set_controller(controller)

		gtk.main()

if __name__ == '__main__':
	try:
		app = App()
	except KeyboardInterrupt:
		logger.debug('Interrupted by User')
		try:
			sys.exit(0)
		except SystemExit:
			os._exit(0)


