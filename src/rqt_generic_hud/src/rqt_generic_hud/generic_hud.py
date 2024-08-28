import os
import sys
import math
import rospkg
import rospy

from importlib import import_module

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal, Qt
from python_qt_binding.QtWidgets import QWidget

from rqt_generic_hud.generic_hud_options import SimpleSettingsDialog

class GenericHUD(Plugin):
	_draw = Signal()

	def __init__(self, context):
		super(GenericHUD, self).__init__(context)
		# Give QObjects reasonable names
		self.setObjectName('GenericHUD')
		rp = rospkg.RosPack()

		# Process standalone plugin command-line arguments
		#from argparse import ArgumentParser
		#parser = ArgumentParser()
		# Add argument(s) to the parser.
		#parser.add_argument("-q", "--quiet", action="store_true",
		#              dest="quiet",
		#              help="Put plugin in silent mode")
		#args, unknowns = parser.parse_known_args(context.argv())
		#if not args.quiet:
		#    print 'arguments: ', args
		#    print 'unknowns: ', unknowns

		# Create QWidget
		self._widget = QWidget()
		# Get path to UI file which is a sibling of this file
		# in this example the .ui and .py file are in the same folder
		ui_file = os.path.join(rp.get_path('rqt_generic_hud'), 'resource', 'GenericHUD.ui')
		# Extend the widget with all attributes and children from UI file
		loadUi(ui_file, self._widget)
		# Give QObjects reasonable names
		self._widget.setObjectName('GenericHUD')
		# Show _widget.windowTitle on left-top of each plugin (when
		# it's set in _widget). This is useful when you open multiple
		# plugins at once. Also if you open multiple instances of your
		# plugin at once, these lines add number to make it easy to
		# tell from pane to pane.
		if context.serial_number() > 1:
			self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
		# Add widget to the user interface
		context.add_widget(self._widget)

		self.sub = None
		self.topic_name = ""
		self.topic_type = ""
		self.topic_content = ""
		self.val_min = 0.0
		self.val_max = 1.0
		self.val_percent = 0.0
		self.display_mode = "Vertical"

		self._draw.connect(self.update_display)

	def shutdown_plugin(self):
		if self.sub is not None:
			self.sub.unregister()

	def save_settings(self, plugin_settings, instance_settings):
		instance_settings.set_value("topic_name", self.topic_name)
		instance_settings.set_value("topic_type", self.topic_type)
		instance_settings.set_value("topic_content", self.topic_content)
		instance_settings.set_value("val_min", self.val_min)
		instance_settings.set_value("val_max", self.val_max)
		instance_settings.set_value("display_mode", self.display_mode)

	def restore_settings(self, plugin_settings, instance_settings):
		self.topic_name = instance_settings.value("topic_name")
		self.topic_type = instance_settings.value("topic_type")
		self.topic_content = instance_settings.value("topic_content")
		try:
			self.val_min = float(instance_settings.value("val_min"))
			self.val_max = float(instance_settings.value("val_max"))
		except TypeError:
			self.val_min = 0.0
			self.val_max = 1.0

		self.set_display_mode(str(instance_settings.value("display_mode")))

		if self.topic_name and self.topic_type and self.topic_content:
			self.sub = rospy.Subscriber(self.topic_name, self.get_topic_class_from_type(self.topic_type), self.sub_callback)

	def trigger_configuration(self):
		self.open_settings_dialog()

	def getKey(self,item):
		return item[0]

	def get_topic_class_from_type(self, msg_type):
		connection_header = msg_type.split("/")
		ros_pkg = connection_header[0] + ".msg"
		msg_type = connection_header[1]

		msg_class = getattr(import_module(ros_pkg), msg_type)

		return msg_class

	def get_topic_type(self, name):
		topics = sorted(rospy.get_published_topics(), key=self.getKey)
		topic_names, topic_types = zip(*topics)
		topic_type = topic_types[topic_names.index(name)]

		msg_class = self.get_topic_class_from_type(topic_type)

		return topic_type, msg_class

	def recursive_topic_content(self, msg_in, content):
		attr = None
		subcontent = content.split('/',1)

		if len(subcontent) > 1:
			attr = self.recursive_topic_content(getattr(msg_in, subcontent[0]), subcontent[1])
		else:
			attr = getattr(msg_in, content)

		return attr

	def sub_callback(self, msg_in):
		val = 0.0

		try:
			val = float(self.recursive_topic_content(msg_in, self.topic_content))
		except AttributeError as e:
			rospy.logwarn("AttributeError: " + str(e))
			self.sub.unregister()
		except TypeError as e:
			rospy.logwarn("Unable to display " + str(getattr(msg_in, self.topic_content).__class__.__name__) + " as a percentage")
			self.sub.unregister()

		val_norm = (val - self.val_min) / (self.val_max - self.val_min)
		self.val_percent = int(100*val_norm)

		self._draw.emit()

	def update_display(self):
		self._widget.progress_bar_status.setValue(self.val_percent)
		self._widget.label_display.setText(str(self.val_percent) + "%")

	def open_settings_dialog(self):
		"""Present the user with a dialog for choosing the topic to view,
		the data type, and other settings used to generate the HUD.
		This displays a SimpleSettingsDialog asking the user to choose
		the settings as desired.

		This method is blocking"""

		dialog = SimpleSettingsDialog(title='HUD Options')
		dialog.add_topic_list("topic_list", str(self.topic_name), "Topics")
		dialog.add_combobox_empty("content_list", "Contents", self.topic_content)
		dialog.add_lineedit("val_min", str(self.val_min), "Minimum")
		dialog.add_lineedit("val_max", str(self.val_max), "Maximum")
		dialog.add_combobox("display_mode", ['Vertical', 'Horizontal'], self.display_mode, "Orientation")

		settings = dialog.get_settings();
		if settings is not None:
			for s in settings:
				if s[0] == "topic_list":
					self.topic_name = str(s[1])
				elif s[0] == "content_list":
					self.topic_content = str(s[1])
				elif s[0] == "val_min":
					self.val_min = float(s[1])
				elif s[0] == "val_max":
					self.val_max = float(s[1])
				elif s[0] == "display_mode":
					self.set_display_mode(s[1])

			if self.topic_name and self.topic_content:
				self.topic_type, msg_class = self.get_topic_type(self.topic_name)
				self.sub = rospy.Subscriber(self.topic_name, msg_class, self.sub_callback)

		self._draw.emit()

	def set_display_mode(self, mode):
		if(mode == 'Horizontal'):
			self.display_mode = 'Horizontal'
			self._widget.progress_bar_status.setOrientation(Qt.Horizontal)
		else:
			self.display_mode = 'Vertical'
			self._widget.progress_bar_status.setOrientation(Qt.Vertical)
