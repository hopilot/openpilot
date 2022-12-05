#include "selfdrive/ui/qt/onroad.h"

#include <QDebug>
#include <QFileInfo>
#include <QDateTime>

#include "selfdrive/common/timing.h"
#include "selfdrive/ui/paint.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/api.h"
#include "selfdrive/ui/qt/widgets/input.h"
#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map.h"
#endif

OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  main_layout->setMargin(bdr_s);
  QStackedLayout *stacked_layout = new QStackedLayout;
  stacked_layout->setStackingMode(QStackedLayout::StackAll);
  main_layout->addLayout(stacked_layout);

  nvg = new NvgWindow(VISION_STREAM_RGB_ROAD, this);

  QWidget * split_wrapper = new QWidget;
  split = new QHBoxLayout(split_wrapper);
  split->setContentsMargins(0, 0, 0, 0);
  split->setSpacing(0);
  split->addWidget(nvg);

  stacked_layout->addWidget(split_wrapper);

  alerts = new OnroadAlerts(this);
  alerts->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  stacked_layout->addWidget(alerts);

  // setup stacking order
  alerts->raise();

  setAttribute(Qt::WA_OpaquePaintEvent);
  QObject::connect(this, &OnroadWindow::updateStateSignal, this, &OnroadWindow::updateState);
  QObject::connect(this, &OnroadWindow::offroadTransitionSignal, this, &OnroadWindow::offroadTransition);
}

void OnroadWindow::updateState(const UIState &s) {
  QColor bgColor = bg_colors[s.status];
  Alert alert = Alert::get(*(s.sm), s.scene.started_frame);
  if (s.sm->updated("controlsState") || !alert.equal({})) {
    if (alert.type == "controlsUnresponsive") {
      bgColor = bg_colors[STATUS_ALERT];
    }
    if (!QUIState::ui_state.is_OpenpilotViewEnabled) {
      // opkr
      if (QFileInfo::exists("/data/log/error.txt") && s.scene.show_error && !s.scene.tmux_error_check) {
        QFileInfo fileInfo;
        fileInfo.setFile("/data/log/error.txt");
        QDateTime modifiedtime = fileInfo.lastModified();
        QString modified_time = modifiedtime.toString("yyyy-MM-dd hh:mm:ss ");
        const std::string txt = util::read_file("/data/log/error.txt");
        if (RichTextDialog::alert(modified_time + QString::fromStdString(txt), this)) {
          QUIState::ui_state.scene.tmux_error_check = true;
        }
      }
	  alerts->updateAlert(alert, bgColor);
    }
  }
  if (bg != bgColor) {
    // repaint border
    bg = bgColor;
    update();
  }
}

void OnroadWindow::mousePressEvent(QMouseEvent* e) {
  // propagation event to parent(HomeWindow)
  QWidget::mousePressEvent(e);

  if ((map_overlay_btn.ptInRect(e->x(), e->y()) || map_btn.ptInRect(e->x(), e->y()) || map_return_btn.ptInRect(e->x(), e->y()) || 
    rec_btn.ptInRect(e->x(), e->y()) || laneless_btn.ptInRect(e->x(), e->y()) || monitoring_btn.ptInRect(e->x(), e->y()) || speedlimit_btn.ptInRect(e->x(), e->y()) ||
    stockui_btn.ptInRect(e->x(), e->y()) || tuneui_btn.ptInRect(e->x(), e->y()) || mapbox_btn.ptInRect(e->x(), e->y()) || QUIState::ui_state.scene.map_on_top || 
    QUIState::ui_state.scene.live_tune_panel_enable)) {return;}
  if (map != nullptr) {
    bool sidebarVisible = geometry().x() > 0;
    map->setVisible(!sidebarVisible && !map->isVisible());
    if (map->isVisible()) {
      QUIState::ui_state.scene.mapbox_running = true;
    } else {
      QUIState::ui_state.scene.mapbox_running = false;
    }
  }
}

void OnroadWindow::offroadTransition(bool offroad) {
#ifdef ENABLE_MAPS
  if (!offroad) {
    QString token = QString::fromStdString(Params().get("dp_mapbox_token_sk"));
    if (map == nullptr && !token.isEmpty() && Params().getBool("MapboxEnabled")) {
      QMapboxGLSettings settings;

      // // Valid for 4 weeks since we can't swap tokens on the fly
      // QString token = MAPBOX_TOKEN.isEmpty() ? CommaApi::create_jwt({}, 4 * 7 * 24 * 3600) : MAPBOX_TOKEN;

      if (!Hardware::PC()) {
        settings.setCacheDatabasePath("/data/mbgl-cache.db");
      }
      settings.setApiBaseUrl(MAPS_HOST);
      settings.setCacheDatabaseMaximumSize(20 * 1024 * 1024);
      settings.setAccessToken(token.trimmed());

      MapWindow * m = new MapWindow(settings);
      m->setFixedWidth(topWidget(this)->width() / 2);
      QObject::connect(this, &OnroadWindow::offroadTransitionSignal, m, &MapWindow::offroadTransition);
      split->addWidget(m, 0, Qt::AlignRight);
      map = m;
    }
  }
#endif

  alerts->updateAlert({}, bg);

  // update stream type
  bool wide_cam = Hardware::TICI() && Params().getBool("EnableWideCamera");
  nvg->setStreamType(wide_cam ? VISION_STREAM_RGB_WIDE_ROAD : VISION_STREAM_RGB_ROAD);
}

void OnroadWindow::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.fillRect(rect(), QColor(bg.red(), bg.green(), bg.blue(), 255));
}

// ***** onroad widgets *****

// OnroadAlerts
void OnroadAlerts::updateAlert(const Alert &a, const QColor &color) {
  if (!alert.equal(a) || color != bg) {
    alert = a;
    bg = color;
    update();
  }
}

void OnroadAlerts::paintEvent(QPaintEvent *event) {
  if (alert.size == cereal::ControlsState::AlertSize::NONE) {
    return;
  }
  static std::map<cereal::ControlsState::AlertSize, const int> alert_sizes = {
    {cereal::ControlsState::AlertSize::SMALL, 271},
    {cereal::ControlsState::AlertSize::MID, 420},
    {cereal::ControlsState::AlertSize::FULL, height()},
  };
  int h = alert_sizes[alert.size];
  QRect r = QRect(0, height() - h, width(), h);

  QPainter p(this);

  // draw background + gradient
  p.setPen(Qt::NoPen);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

  p.setBrush(QBrush(bg));
  p.drawRect(r);

  QLinearGradient g(0, r.y(), 0, r.bottom());
  g.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.05));
  g.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0.35));

  p.setCompositionMode(QPainter::CompositionMode_DestinationOver);
  p.setBrush(QBrush(g));
  p.fillRect(r, g);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

  // text
  const QPoint c = r.center();
  p.setPen(QColor(0xff, 0xff, 0xff));
  p.setRenderHint(QPainter::TextAntialiasing);
  if (alert.size == cereal::ControlsState::AlertSize::SMALL) {
    configFont(p, "Open Sans", 74, "SemiBold");
    p.drawText(r, Qt::AlignCenter, alert.text1);
  } else if (alert.size == cereal::ControlsState::AlertSize::MID) {
    configFont(p, "Open Sans", 88, "Bold");
    p.drawText(QRect(0, c.y() - 125, width(), 150), Qt::AlignHCenter | Qt::AlignTop, alert.text1);
    configFont(p, "Open Sans", 66, "Regular");
    p.drawText(QRect(0, c.y() + 21, width(), 90), Qt::AlignHCenter, alert.text2);
  } else if (alert.size == cereal::ControlsState::AlertSize::FULL) {
    bool l = alert.text1.length() > 15;
    configFont(p, "Open Sans", l ? 132 : 177, "Bold");
    p.drawText(QRect(0, r.y() + (l ? 240 : 270), width(), 600), Qt::AlignHCenter | Qt::TextWordWrap, alert.text1);
    configFont(p, "Open Sans", 88, "Regular");
    p.drawText(QRect(0, r.height() - (l ? 361 : 420), width(), 300), Qt::AlignHCenter | Qt::TextWordWrap, alert.text2);
  }
}

void NvgWindow::initializeGL() {
  CameraViewWidget::initializeGL();
  qInfo() << "OpenGL version:" << QString((const char*)glGetString(GL_VERSION));
  qInfo() << "OpenGL vendor:" << QString((const char*)glGetString(GL_VENDOR));
  qInfo() << "OpenGL renderer:" << QString((const char*)glGetString(GL_RENDERER));
  qInfo() << "OpenGL language version:" << QString((const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

  ui_nvg_init(&QUIState::ui_state);
  prev_draw_t = millis_since_boot();
  setBackgroundColor(bg_colors[STATUS_DISENGAGED]);
}

void NvgWindow::drawLaneLines(QPainter &painter, const UIState *s) {
  painter.save();

  const UIScene &scene = s->scene;
  SubMaster &sm = *(s->sm);

  // paint blindspot line
  painter.setBrush( QColor::fromRgbF(1.0, 0.1, 0.1, 0.2) );
  if(scene.leftblindspot) {
    painter.drawPolygon(scene.lane_line_vertices[1]);
    painter.drawPolygon(scene.lane_blindspot_vertices[0]);
  }
  if(scene.rightblindspot) {
    painter.drawPolygon(scene.lane_line_vertices[2]);
    painter.drawPolygon(scene.lane_blindspot_vertices[1]);
  }

  int steerOverride = scene.car_state.getSteeringPressed();
  float red_lvl_line = 0;
  float green_lvl_line = 0;

  // lanelines
  for (int i = 0; i < std::size(scene.lane_line_vertices); i++) {
    if (scene.lane_line_probs[i] > 0.4){
      red_lvl_line = 1.0 - ((scene.lane_line_probs[i] - 0.4) * 2.5);
      green_lvl_line = 1.0;
    } else {
      red_lvl_line = 1.0;
      green_lvl_line = 1.0 - ((0.4 - scene.lane_line_probs[i]) * 2.5);
    }
    painter.setBrush(QColor::fromRgbF(red_lvl_line, green_lvl_line, 0, std::clamp<float>(scene.lane_line_probs[i], 0.0, 0.7)));
    painter.drawPolygon(scene.lane_line_vertices[i]);
  }

  for (int i = 0; i < std::size(scene.lane_line_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, std::clamp<float>(scene.lane_line_probs[i], 0.0, 0.7)));
    painter.drawPolygon(scene.lane_line_vertices[i]);
  }

  // road edges
  for (int i = 0; i < std::size(scene.road_edge_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(0.8, 0.1, 0.1, std::clamp<float>(1.0 - scene.road_edge_stds[i], 0.0, 0.8)));
    painter.drawPolygon(scene.road_edge_vertices[i]);
  }

  // paint path
  QLinearGradient bg(0, height(), 0, height() / 4);
  float start_hue, end_hue;

  if (scene.engageable) {
    const auto &acceleration = sm["modelV2"].getModelV2().getAcceleration();
    float acceleration_future = 0;
    if (acceleration.getZ().size() > 16) {
      acceleration_future = acceleration.getX()[16];  // 2.5 seconds
    }
    start_hue = 60;
    // speed up: 120, slow down: 0
    end_hue = fmax(fmin(start_hue + acceleration_future * 45, 148), 0);

    // FIXME: painter.drawPolygon can be slow if hue is not rounded
    end_hue = int(end_hue * 100 + 0.5) / 100;

    if (scene.steeringPress) {
      // The user is applying torque to the steering wheel
      bg.setColorAt(0.0, steeringpressedColor(100));
      bg.setColorAt(0.5, steeringpressedColor(50));
      bg.setColorAt(1.0, steeringpressedColor(0));
    } else if (scene.gasPress) {
      bg.setColorAt(0.0, overrideColor(100));
      bg.setColorAt(0.5, overrideColor(50));
      bg.setColorAt(1.0, overrideColor(0));
    } else if (!scene.lateralPlan.lanelessModeStatus) {
      bg.setColorAt(0.0, QColor::fromHslF(start_hue / 360., 0.97, 0.56, 0.4));
      bg.setColorAt(0.5, QColor::fromHslF(end_hue / 360., 1.0, 0.68, 0.35));
      bg.setColorAt(1.0, QColor::fromHslF(end_hue / 360., 1.0, 0.68, 0.0));
    } else {
      bg.setColorAt(0.0, QColor::fromHslF(148 / 360., 0.94, 0.51, 0.4));
      bg.setColorAt(0.5, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.35));
      bg.setColorAt(1.0, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.0));
    }
  } else {
    bg.setColorAt(0.0, whiteColor(100));
    bg.setColorAt(0.5, whiteColor(50));
    bg.setColorAt(1.0, whiteColor(0));
  }
  painter.setBrush(bg);
  painter.drawPolygon(scene.track_vertices);

  painter.restore();
}

void NvgWindow::paintGL() {
  CameraViewWidget::paintGL();
  ui_draw(&QUIState::ui_state, width(), height());

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(Qt::NoPen);	

  drawLaneLines(painter, &QUIState::ui_state);

  double cur_draw_t = millis_since_boot();
  double dt = cur_draw_t - prev_draw_t;
  if (dt > 66) {
    // warn on sub 15fps
    LOGW("slow frame time: %.2f", dt);
  }
  prev_draw_t = cur_draw_t;
}

void NvgWindow::showEvent(QShowEvent *event) {
  CameraViewWidget::showEvent(event);
  ui_update_params(&QUIState::ui_state);
  prev_draw_t = millis_since_boot();
}
