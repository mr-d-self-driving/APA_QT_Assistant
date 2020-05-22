/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

//! [0]
/* Add C includes here */

#if defined __cplusplus
/* Add C++ includes here */
# include <cassert>
# include <iostream>
# include <sstream>
# include <fstream>

# include <vector>
# include <list>
# include <map>
# include <set>
# include <queue>
# include <memory>

# include <cmath>
# include <ctime>
# include <utility>
# include <functional>
# include <string>
# include <cstdlib>
# include <cstdint>
# include <limits>
# include <valarray>
# include <unordered_set>
# include <unordered_map>
# include <algorithm>
# include <random>
# include <mutex>
# include <type_traits>
# include <chrono>
# include <iomanip>

//# include <QMainWindow>
# include <QApplication>
# include <QObject>
# include <qglobal.h>
# include <QDir>
# include <QIntValidator>
# include <QToolTip>
# include <QDebug>
# include <QMainWindow>
# include <QDialog>
# include <QFileDialog>
# include <QPushButton>
# include <QLabel>
# include <QLineEdit>
# include <QTimer>
# include <QTextCodec>
# include <QThread>
# if (QT_VERSION > QT_VERSION_CHECK(5,0,0))
# include <QtWidgets>
# endif

# include <QGraphicsScene>
# include <QGraphicsItem>
# include <QGraphicsPixmapItem>
# include <QGraphicsSceneWheelEvent>


//#include <Eigen/Core>
//#include <Eigen/Dense>

//#include <boost/archive/binary_oarchive.hpp>
//#include <boost/archive/binary_iarchive.hpp>
//#include <boost/concept_check.hpp>
//#include <boost/serialization/access.hpp>
//#include <boost/serialization/vector.hpp>
//#include <boost/serialization/utility.hpp>
//#include <boost/graph/adjacency_list.hpp>
//#include <boost/graph/graph_traits.hpp>
#endif
//! [0]
