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
#    The configuration class for ParaNut is implemented in this file
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

## Unit class provides methods to get and set unit attributes.
class Unit:

	## Writes the "config.mk" file.
	#
	#  @param self			The object pointer.
	def _write_unit(self, path = "."):
		with open(f'{path}/config.mk', "a") as config:
			for key in self.__dict__.keys():
				attr = getattr(self, key)

				config.write('\n'.join(attr.get('description')))
				config.write('\n')
				config.write(str(attr.get('name')))
				config.write(' ?= ')
				config.write(str(attr.get('value')))
				config.write('\n\n')
	
	def get_attributes(self):
		return list(self.__dict__.keys())

	## Converts the Unit attribute variables into a list.
	#
	#  @param self			The object pointer.
	#  @return				A list of lables.
	def get_labels(self):
		labels = list()
		for key in self.__dict__.keys():
			attr = getattr(self, key)
			labels.append(str(attr.get('name')))
		return labels

	## Converts the Unit attribute values into a list. 
	#
	#  @param self			The object pointer.
	#  @return 				A list of valuse.
	def get_values(self):
		values = list()
		
		for key in self.__dict__.keys():
			attr = getattr(self, key)
			values.append(str(attr.get('value')))
		return values
		
	## Gets the label of the Units attribute.
	#
	#  @param self			The object pointer.
	#  @param attr			Attribute of the Unit.
	#  @return				A label as string.
	def get_label(self, attr: str):
		assert isinstance(attr, str), 'Wrong type'
		assert attr, 'No data'

		attrubute = getattr(self, attr)
		label = str(attribute.get('name'))
		
		assert label, 'No label'
		
		return label
		
	## Gets the value of the Units attribute.
	#
	#  @param self			The object pointer.
	#  @param attr 			Attribute of the Unit.
	#  @return				The value as string
	def get_value(self, attr: str):
		assert isinstance(attr, str), 'Wrong type'
		assert attr, 'No data'

		attribute = getattr(self, attr)
		value = str(attribute.get('value'))
		
		assert value, 'No value'
		
		return value

	## Gets the value of the Units attribute.
	#
	#  @param self			The object pointer.
	#  @param attr 			Atribute of the Unit.
	#  @return				The value as string
	def set_value(self, attr: str, value: str):
		attribute = getattr(self, attr)
		attribute.update(f'"value": "{value}"')
		
## The NUT class.
class NUT(Unit):
	#  @param self			The object pointer.
	def __init__(self, **kwargs):
		self.mtimer_timebase_us = kwargs.get('mtimer_timebase_us')
		self.mtimer_addr		= kwargs.get('mtimer_addr')
		self.reset_addr			= kwargs.get('reset_addr')
		self.cpu_cores_ld		= kwargs.get('cpu_cores_ld')
		self.cpu_cap1_cores		= kwargs.get('cpu_cap1_cores')
		self.mem_size			= kwargs.get('mem_size')
		self.ex_int				= kwargs.get('ex_int')
		self.sim_clk_speed		= kwargs.get('sim_clk_speed')
		self.sim_max_periphery	= kwargs.get('sim_max_periphery')

## The EXU class.
class EXU(Unit):
	#  @param self			The object pointer.
	def __init__(self, **kwargs):
		self.m_extension		= kwargs.get('m_extension')
		self.a_extension		= kwargs.get('a_extension')
		self.priv_levels		= kwargs.get('priv_levels')
		self.perfcount_enable	= kwargs.get('perfcount_enable')
		self.perfcounter_bits	= kwargs.get('perfcounter_bits')
		self.perfcounters_ld	= kwargs.get('perfcounters_ld')

## The MEMU class.
class MEMU(Unit):
	#  @param self			The object pointer.
	def __init__(self, **kwargs):
		self.cache_banks_ld		= kwargs.get('cache_banks_ld')
		self.cache_sets_ld		= kwargs.get('cache_sets_ld')
		self.cache_ways_ld		= kwargs.get('cache_ways_ld')
		self.cache_replace_lru	= kwargs.get('cache_replace_lru')
		self.arbiter_method		= kwargs.get('arbiter_method')
		self.busif_width		= kwargs.get('busif_width')

## The MMU class.
class MMU(Unit):
	#  @param self			The object pointer.
	def __init__(self, **kwargs):
		self.tlb_enable			= kwargs.get('tlb_enable')
		self.tlb_entries_ld		= kwargs.get('tlb_entries_ld')

## The IFU class.
class IFU(Unit):
	#  @param self			The object pointer.
	def __init__(self, **kwargs):
		self.ibuf_size_ld		= kwargs.get('ibuf_size_ld')

## The LSU class.
class LSU(Unit):
	#  @param self			The object pointer.
	def __init__(self, **kwargs):
		self.wbuf_size_ld		= kwargs.get('wbuf_size_ld')
