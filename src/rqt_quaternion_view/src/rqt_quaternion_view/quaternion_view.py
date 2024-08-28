import os
import sys
import math
import rospkg
import rospy

from importlib import import_module

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal, Qt
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D

from tf import transformations as tft
import math

from geometry_msgs.msg import Quaternion
from rqt_quaternion_view.quaternion_view_options import SimpleSettingsDialog

class QuaternionView(Plugin):
	_draw = Signal()

	def __init__(self, context):
		super(QuaternionView, self).__init__(context)
		# Give QObjects reasonable names
		self.setObjectName('QuaternionView')
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
		ui_file = os.path.join(rp.get_path('rqt_quaternion_view'), 'resource', 'QuaternionView.ui')
		# Extend the widget with all attributes and children from UI file
		loadUi(ui_file, self._widget)
		# Give QObjects reasonable names
		self._widget.setObjectName('QuaternionView')
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
		self.refresh_rate = 5.0
		self.refresh_period = rospy.Duration(0)
		self.time_last = rospy.Time(0)
		self.manual_mode = True
		self.val = Quaternion(0.0,0.0,0.0,1.0)

		self.update_refresh_period()
		self.set_manual_mode(self.manual_mode)

		self.plot_3d_figure = Figure()
		self.plot_3d_figure.patch.set_facecolor('white')
		self.plot_3d_canvas = FigureCanvas(self.plot_3d_figure)
		self.plot_3d_toolbar = NavigationToolbar(self.plot_3d_canvas, self._widget.widget_plot_3d)
		self.plot_3d_ax = self.plot_3d_figure.add_subplot(1,1,1, projection='3d')
		self.set_plot_layout(self._widget.widget_plot_3d, self.plot_3d_toolbar, self.plot_3d_canvas)
		self.plot_3d_ax.set_title(self.topic_name)
		self.plot_3d_ax.set_xlabel('X')
		self.plot_3d_ax.set_ylabel('Y')
		self.plot_3d_ax.set_zlabel('Z')

		self._draw.connect(self.update_display)
		self.update_display()

	def shutdown_plugin(self):
		if self.sub is not None:
			self.sub.unregister()

	def save_settings(self, plugin_settings, instance_settings):
		instance_settings.set_value("topic_name", self.topic_name)
		instance_settings.set_value("topic_type", self.topic_type)
		instance_settings.set_value("topic_content", self.topic_content)
		instance_settings.set_value("refresh_rate", self.refresh_rate)
		instance_settings.set_value("manual_mode", str(self.manual_mode))

	def restore_settings(self, plugin_settings, instance_settings):
		topic_name = instance_settings.value("topic_name")
		topic_type = instance_settings.value("topic_type")
		topic_content = instance_settings.value("topic_content")
		manual_mode = instance_settings.value("manual_mode")
		refresh_rate = instance_settings.value("refresh_rate")

		if (topic_name is not None) and (topic_type is not None) and (topic_content is not None) and (manual_mode is not None) and (refresh_rate is not None):

			self.topic_name = str(topic_name)
			self.topic_type = str(topic_type)
			self.topic_content = str(topic_content)
			self.manual_mode = (str(manual_mode).lower() == "true")
			self.refresh_rate = self.parse_float(refresh_rate, default=5.0)

			self.update_refresh_period()

			self.set_manual_mode(self.manual_mode)

			if (not self.manual_mode) and self.topic_name and self.topic_type and self.topic_content:
				self.sub = rospy.Subscriber(self.topic_name, self.get_topic_class_from_type(self.topic_type), self.sub_callback)

	def trigger_configuration(self):
		self.open_settings_dialog()

	def set_plot_layout(self, widget, toolbar, canvas):
		layout = QVBoxLayout()
		layout.addWidget(toolbar)
		layout.addWidget(canvas)
		widget.setLayout(layout)

	def clear_plot(self):
		# Discards the old graph data
		artists = self.plot_3d_ax.lines + self.plot_3d_ax.collections

		for artist in artists:
			artist.remove()

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
		now = rospy.Time.now()

		if now > self.time_last:
			# Rate-limit refreshes
			if now - self.time_last > self.refresh_period:
				self.val = Quaternion(0.0,0.0,0.0,1.0)

				try:
					new_val = self.recursive_topic_content(msg_in, self.topic_content)
					if type(new_val) is Quaternion:
						val = new_val
					else:
						raise TypeError
				except AttributeError as e:
					rospy.logwarn("AttributeError: " + str(e))
					self.sub.unregister()
				except TypeError as e:
					rospy.logwarn("Unable to display " + str(getattr(msg_in, self.topic_content).__class__.__name__) + " as a quaternion")
					self.sub.unregister()

				self.val = val

				self.time_last = now
				self._draw.emit()
		else:
			# Timestep backwards, reset
			self.time_last = rospy.Time(0)

	def normalize_tf_quaternion(self,q):
		d = math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])

		if d > 0.0:
			q[0] = q[0]/d
			q[1] = q[1]/d
			q[2] = q[2]/d
			q[3] = q[3]/d
		else:
			#Invalid quaternion, reset
			q[0] = 0.0
			q[1] = 0.0
			q[2] = 0.0
			q[3] = 1.0

		return q

	def update_display(self):
		self.clear_plot()

		x = [1.0,0.0,0.0,0.0]
		x2 = [0.8,0.0,0.0,0.0]
		z = [0.8,0.0,0.1,0.0]
		q = self.normalize_tf_quaternion([self.val.x, self.val.y, self.val.z,self.val.w])

		xr = tft.quaternion_multiply(tft.quaternion_multiply(q,x),tft.quaternion_inverse(q))
		x2r = tft.quaternion_multiply(tft.quaternion_multiply(q,x2),tft.quaternion_inverse(q))
		zr = tft.quaternion_multiply(tft.quaternion_multiply(q,z),tft.quaternion_inverse(q))

		self.plot_3d_ax.plot([0.0,0.5], [0.0,0.0], [0.0,0.0], 'r-')
		self.plot_3d_ax.plot([0.0,0.0], [0.0,0.5], [0.0,0.0], 'g-')
		self.plot_3d_ax.plot([0.0,0.0], [0.0,0.0], [0.0,0.5], 'b-')
		self.plot_3d_ax.plot([0.0,xr[0]], [0.0,xr[1]], [0.0,xr[2]], 'k-', linewidth=2)
		self.plot_3d_ax.plot([zr[0],xr[0]], [zr[1],xr[1]], [zr[2],xr[2]], 'k-', linewidth=2)
		self.plot_3d_ax.plot([zr[0],x2r[0]], [zr[1],x2r[1]], [zr[2],x2r[2]], 'k-', linewidth=2)

		self.plot_3d_ax.set_aspect('equal', 'box')
		self.plot_3d_ax.set_xlim(-1.0, 1.0)
		self.plot_3d_ax.set_ylim(-1.0, 1.0)
		self.plot_3d_ax.set_zlim(-1.0, 1.0)

		self.plot_3d_canvas.draw()

		if not self.manual_mode:
			self.update_normalized_displays()

	def parse_float(self, text, default=0.0):
		val = default
		try:
			val = float(text)
		except (ValueError, TypeError):
			pass

		return val

	def manual_update(self, _=None, new_val=None, update_euler=True):
		if type(new_val) is Quaternion:
			self.val = new_val
		else:
			self.val = Quaternion(self.parse_float(self._widget.input_q_x.text()),
								  self.parse_float(self._widget.input_q_y.text()),
								  self.parse_float(self._widget.input_q_z.text()),
								  self.parse_float(self._widget.input_q_w.text()))

		if update_euler:
			q = [self.val.x, self.val.y, self.val.z,self.val.w]
			e = tft.euler_from_quaternion(q, axes='rzyx')

			self._widget.input_e_r.setText('%.5f' % e[0])
			self._widget.input_e_p.setText('%.5f' % e[0])
			self._widget.input_e_y.setText('%.5f' % e[0])
			self._widget.input_e_r_deg.setText('%.5f' % math.degrees(e[2]))
			self._widget.input_e_p_deg.setText('%.5f' % math.degrees(e[1]))
			self._widget.input_e_y_deg.setText('%.5f' % math.degrees(e[0]))

		self._draw.emit()

	def manual_update_rpy(self, _=None, new_val=None, update_euler_degrees=True):
		yaw = 0.0
		pitch = 0.0
		roll = 0.0

		if new_val is None:
			yaw = self.parse_float(self._widget.input_e_y.text())
			pitch = self.parse_float(self._widget.input_e_p.text())
			roll = self.parse_float(self._widget.input_e_r.text())
		else:
			yaw = new_val[0]
			pitch = new_val[1]
			roll = new_val[2]

		q = tft.quaternion_from_euler(yaw, pitch, roll, axes='rzyx')

		self._widget.input_q_w.setText('%.5f' % q[3])
		self._widget.input_q_x.setText('%.5f' % q[0])
		self._widget.input_q_y.setText('%.5f' % q[1])
		self._widget.input_q_z.setText('%.5f' % q[2])

		if update_euler_degrees:
			self._widget.input_e_r_deg.setText('%.5f' % math.degrees(roll))
			self._widget.input_e_p_deg.setText('%.5f' % math.degrees(pitch))
			self._widget.input_e_y_deg.setText('%.5f' % math.degrees(yaw))

		self.manual_update(new_val=Quaternion(q[0], q[1], q[2], q[3]), update_euler=False)

	def manual_update_rpy_deg(self, _=None):
		yaw = math.radians(self.parse_float(self._widget.input_e_y_deg.text()))
		pitch = math.radians(self.parse_float(self._widget.input_e_p_deg.text()))
		roll = math.radians(self.parse_float(self._widget.input_e_r_deg.text()))

		self.manual_update_rpy(new_val=[yaw,pitch,roll], update_euler_degrees=False)

	def update_normalized_displays(self):
		q = self.normalize_tf_quaternion([self.val.x, self.val.y, self.val.z,self.val.w])
		e = tft.euler_from_quaternion(q, axes='szyx')

		self._widget.input_q_w.setText('%.5f' % q[3])
		self._widget.input_q_x.setText('%.5f' % q[0])
		self._widget.input_q_y.setText('%.5f' % q[1])
		self._widget.input_q_z.setText('%.5f' % q[2])
		self._widget.input_e_r.setText('%.5f' % e[2])
		self._widget.input_e_p.setText('%.5f' % e[1])
		self._widget.input_e_y.setText('%.5f' % e[0])
		self._widget.input_e_r_deg.setText('%.5f' % math.degrees(e[2]))
		self._widget.input_e_p_deg.setText('%.5f' % math.degrees(e[1]))
		self._widget.input_e_y_deg.setText('%.5f' % math.degrees(e[0]))

	def set_manual_mode(self, manual):
		if manual:
			print("Manual mode")
			if self.sub is not None:
				self.sub.unregister()

			self._widget.input_q_w.textEdited.connect(self.manual_update)
			self._widget.input_q_x.textEdited.connect(self.manual_update)
			self._widget.input_q_y.textEdited.connect(self.manual_update)
			self._widget.input_q_z.textEdited.connect(self.manual_update)
			self._widget.input_e_r.textEdited.connect(self.manual_update_rpy)
			self._widget.input_e_p.textEdited.connect(self.manual_update_rpy)
			self._widget.input_e_y.textEdited.connect(self.manual_update_rpy)
			self._widget.input_e_r_deg.textEdited.connect(self.manual_update_rpy_deg)
			self._widget.input_e_p_deg.textEdited.connect(self.manual_update_rpy_deg)
			self._widget.input_e_y_deg.textEdited.connect(self.manual_update_rpy_deg)

			self._widget.input_q_w.returnPressed.connect(self.update_normalized_displays)
			self._widget.input_q_x.returnPressed.connect(self.update_normalized_displays)
			self._widget.input_q_y.returnPressed.connect(self.update_normalized_displays)
			self._widget.input_q_z.returnPressed.connect(self.update_normalized_displays)
			self._widget.input_e_r.returnPressed.connect(self.update_normalized_displays)
			self._widget.input_e_p.returnPressed.connect(self.update_normalized_displays)
			self._widget.input_e_y.returnPressed.connect(self.update_normalized_displays)
			self._widget.input_e_r_deg.returnPressed.connect(self.update_normalized_displays)
			self._widget.input_e_p_deg.returnPressed.connect(self.update_normalized_displays)
			self._widget.input_e_y_deg.returnPressed.connect(self.update_normalized_displays)
		else:
			print("Subscriber mode")
			try:
				self._widget.input_q_w.textEdited.disconnect()
				self._widget.input_q_x.textEdited.disconnect()
				self._widget.input_q_y.textEdited.disconnect()
				self._widget.input_q_z.textEdited.disconnect()
				self._widget.input_e_r.textEdited.disconnect()
				self._widget.input_e_p.textEdited.disconnect()
				self._widget.input_e_y.textEdited.disconnect()
				self._widget.input_e_r_deg.textEdited.disconnect()
				self._widget.input_e_p_deg.textEdited.disconnect()
				self._widget.input_e_y_deg.textEdited.disconnect()

				self._widget.input_q_w.returnPressed.disconnect()
				self._widget.input_q_x.returnPressed.disconnect()
				self._widget.input_q_y.returnPressed.disconnect()
				self._widget.input_q_z.returnPressed.disconnect()
				self._widget.input_e_r.returnPressed.disconnect()
				self._widget.input_e_p.returnPressed.disconnect()
				self._widget.input_e_y.returnPressed.disconnect()
				self._widget.input_e_r_deg.returnPressed.disconnect()
				self._widget.input_e_p_deg.returnPressed.disconnect()
				self._widget.input_e_y_deg.returnPressed.disconnect()
			except TypeError:
				pass

	def update_refresh_period(self):
		self.refresh_period = rospy.Duration( 1.0 / self.refresh_rate )

	def open_settings_dialog(self):
		"""Present the user with a dialog for choosing the topic to view,
		the data type, and other settings used to generate the HUD.
		This displays a SimpleSettingsDialog asking the user to choose
		the settings as desired.

		This method is blocking"""

		dialog = SimpleSettingsDialog(title='Quaternion View Options')
		dialog.add_topic_list("topic_list", str(self.topic_name), "Topics")
		dialog.add_combobox_empty("content_list", "Contents", str(self.topic_content))
		dialog.add_lineedit("refresh_rate", str(self.refresh_rate), "Refresh Rate")
		dialog.add_checkbox("manual_mode", bool(self.manual_mode), "Manual Mode")

		settings = dialog.get_settings();
		if settings is not None:
			for s in settings:
				if s[0] == "topic_list":
					self.topic_name = str(s[1])
				elif s[0] == "content_list":
					self.topic_content = str(s[1])
				elif s[0] == "refresh_rate":
					self.refresh_rate = self.parse_float(s[1])
				elif s[0] == "manual_mode":
					self.manual_mode = bool(s[1])

			self.update_refresh_period()

			if self.manual_mode:
				self.set_manual_mode(True)
			elif self.topic_name and self.topic_content:
				self.topic_type, msg_class = self.get_topic_type(self.topic_name)
				self.sub = rospy.Subscriber(self.topic_name, msg_class, self.sub_callback)

		self._draw.emit()

