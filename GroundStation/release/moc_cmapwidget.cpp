/****************************************************************************
** Meta object code from reading C++ file 'cmapwidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.14.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../cmapwidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'cmapwidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.14.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_CMapWidget_t {
    QByteArrayData data[13];
    char stringdata0[121];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CMapWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CMapWidget_t qt_meta_stringdata_CMapWidget = {
    {
QT_MOC_LITERAL(0, 0, 10), // "CMapWidget"
QT_MOC_LITERAL(1, 11, 7), // "updated"
QT_MOC_LITERAL(2, 19, 0), // ""
QT_MOC_LITERAL(3, 20, 4), // "rect"
QT_MOC_LITERAL(4, 25, 10), // "timerEvent"
QT_MOC_LITERAL(5, 36, 12), // "QTimerEvent*"
QT_MOC_LITERAL(6, 49, 5), // "event"
QT_MOC_LITERAL(7, 55, 11), // "LoadMap_IMG"
QT_MOC_LITERAL(8, 67, 11), // "LoadMap_SQL"
QT_MOC_LITERAL(9, 79, 9), // "SetMapDir"
QT_MOC_LITERAL(10, 89, 3), // "dir"
QT_MOC_LITERAL(11, 93, 19), // "processIncomingData"
QT_MOC_LITERAL(12, 113, 7) // "timeOut"

    },
    "CMapWidget\0updated\0\0rect\0timerEvent\0"
    "QTimerEvent*\0event\0LoadMap_IMG\0"
    "LoadMap_SQL\0SetMapDir\0dir\0processIncomingData\0"
    "timeOut"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CMapWidget[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   49,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    1,   52,    2, 0x0a /* Public */,
       7,    0,   55,    2, 0x08 /* Private */,
       8,    0,   56,    2, 0x08 /* Private */,
       9,    1,   57,    2, 0x08 /* Private */,
      11,    0,   60,    2, 0x08 /* Private */,
      12,    0,   61,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QRect,    3,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   10,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void CMapWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<CMapWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updated((*reinterpret_cast< const QRect(*)>(_a[1]))); break;
        case 1: _t->timerEvent((*reinterpret_cast< QTimerEvent*(*)>(_a[1]))); break;
        case 2: _t->LoadMap_IMG(); break;
        case 3: _t->LoadMap_SQL(); break;
        case 4: _t->SetMapDir((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 5: _t->processIncomingData(); break;
        case 6: _t->timeOut(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (CMapWidget::*)(const QRect & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CMapWidget::updated)) {
                *result = 0;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject CMapWidget::staticMetaObject = { {
    QMetaObject::SuperData::link<QFrame::staticMetaObject>(),
    qt_meta_stringdata_CMapWidget.data,
    qt_meta_data_CMapWidget,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *CMapWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CMapWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CMapWidget.stringdata0))
        return static_cast<void*>(this);
    return QFrame::qt_metacast(_clname);
}

int CMapWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QFrame::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void CMapWidget::updated(const QRect & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
