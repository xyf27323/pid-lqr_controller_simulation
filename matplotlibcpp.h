#pragma once
/*
 * Minimal matplotlib-cpp compatible header (subset).
 *
 * This project originally ships a `matplotlibcpp.h` wrapper, but in some
 * environments that file may be unreadable due to encoding/encryption.
 *
 * This header provides a small subset of the API sufficient for:
 * - backend()
 * - figure_size()
 * - clf()
 * - xlim()/ylim()
 * - title()
 * - plot()
 * - scatter()
 * - grid()
 * - save()
 * - show()
 * - set_aspect_equal()
 *
 * It embeds Python and calls matplotlib.pyplot directly.
 *
 * Build requires Python3 development headers & libs (already in CMakeLists).
 */

// Python headers must be included before any system headers.
#include <Python.h>

#include <algorithm>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace matplotlibcpp {
namespace detail {

static std::string s_backend;

inline void throw_if_pyerr(const std::string& where) {
  if (PyErr_Occurred()) {
    PyErr_Print();
    throw std::runtime_error("Python error at: " + where);
  }
}

struct Interpreter {
  PyObject* plt = nullptr;

  Interpreter() {
#if PY_MAJOR_VERSION >= 3
    wchar_t name[] = L"robot_planner_plot";
    Py_SetProgramName(name);
#else
    char name[] = "robot_planner_plot";
    Py_SetProgramName(name);
#endif
    Py_Initialize();

    // Import matplotlib, set backend if requested, then import pyplot.
    PyObject* matplotlib_name = PyUnicode_FromString("matplotlib");
    if (!matplotlib_name) throw std::runtime_error("failed to create string: matplotlib");
    PyObject* matplotlib = PyImport_Import(matplotlib_name);
    Py_DECREF(matplotlib_name);
    if (!matplotlib) {
      PyErr_Print();
      throw std::runtime_error("Error loading module matplotlib");
    }

    if (!s_backend.empty()) {
      PyObject* res = PyObject_CallMethod(matplotlib, const_cast<char*>("use"),
                                          const_cast<char*>("s"), s_backend.c_str());
      if (!res) {
        Py_DECREF(matplotlib);
        throw_if_pyerr("matplotlib.use()");
      }
      Py_DECREF(res);
    }

    Py_DECREF(matplotlib);

    PyObject* pyplot_name = PyUnicode_FromString("matplotlib.pyplot");
    if (!pyplot_name) throw std::runtime_error("failed to create string: pyplot");
    plt = PyImport_Import(pyplot_name);
    Py_DECREF(pyplot_name);
    if (!plt) {
      PyErr_Print();
      throw std::runtime_error("Error loading module matplotlib.pyplot");
    }
  }

  ~Interpreter() {
    if (plt) Py_DECREF(plt);
    if (Py_IsInitialized()) Py_Finalize();
  }

  static Interpreter& get() {
    static Interpreter instance;
    return instance;
  }
};

inline PyObject* kwargs_from_map(const std::map<std::string, std::string>& keywords) {
  PyObject* kwargs = PyDict_New();
  for (const auto& kv : keywords) {
    PyObject* v = PyUnicode_FromString(kv.second.c_str());
    PyDict_SetItemString(kwargs, kv.first.c_str(), v);
    Py_DECREF(v);
  }
  return kwargs;
}

template <typename Numeric>
inline PyObject* py_list_from_vector(const std::vector<Numeric>& v) {
  PyObject* list = PyList_New(static_cast<Py_ssize_t>(v.size()));
  for (Py_ssize_t i = 0; i < static_cast<Py_ssize_t>(v.size()); ++i) {
    PyObject* num = PyFloat_FromDouble(static_cast<double>(v[static_cast<size_t>(i)]));
    PyList_SetItem(list, i, num);  // steals ref
  }
  return list;
}

}  // namespace detail

inline void backend(const std::string& name) { detail::s_backend = name; }

inline void figure_size(size_t w, size_t h) {
  auto& itp = detail::Interpreter::get();
  const size_t dpi = 100;
  PyObject* figsize = PyTuple_New(2);
  PyTuple_SetItem(figsize, 0, PyFloat_FromDouble(static_cast<double>(w) / dpi));
  PyTuple_SetItem(figsize, 1, PyFloat_FromDouble(static_cast<double>(h) / dpi));

  PyObject* kwargs = PyDict_New();
  PyDict_SetItemString(kwargs, "figsize", figsize);
  PyDict_SetItemString(kwargs, "dpi", PyLong_FromLong(static_cast<long>(dpi)));
  Py_DECREF(figsize);

  PyObject* figure_func = PyObject_GetAttrString(itp.plt, "figure");
  PyObject* args = PyTuple_New(0);
  PyObject* res = PyObject_Call(figure_func, args, kwargs);
  Py_DECREF(figure_func);
  Py_DECREF(args);
  Py_DECREF(kwargs);
  if (!res) detail::throw_if_pyerr("plt.figure()");
  Py_DECREF(res);
}

inline void clf() {
  auto& itp = detail::Interpreter::get();
  PyObject* res = PyObject_CallMethod(itp.plt, const_cast<char*>("clf"), nullptr);
  if (!res) detail::throw_if_pyerr("plt.clf()");
  Py_DECREF(res);
}

inline void xlim(double left, double right) {
  auto& itp = detail::Interpreter::get();
  PyObject* res = PyObject_CallMethod(itp.plt, const_cast<char*>("xlim"),
                                      const_cast<char*>("(dd)"), left, right);
  if (!res) detail::throw_if_pyerr("plt.xlim()");
  Py_DECREF(res);
}

inline void ylim(double bottom, double top) {
  auto& itp = detail::Interpreter::get();
  PyObject* res = PyObject_CallMethod(itp.plt, const_cast<char*>("ylim"),
                                      const_cast<char*>("(dd)"), bottom, top);
  if (!res) detail::throw_if_pyerr("plt.ylim()");
  Py_DECREF(res);
}

inline void title(const std::string& s) {
  auto& itp = detail::Interpreter::get();
  PyObject* res = PyObject_CallMethod(itp.plt, const_cast<char*>("title"),
                                      const_cast<char*>("s"), s.c_str());
  if (!res) detail::throw_if_pyerr("plt.title()");
  Py_DECREF(res);
}

inline void grid(bool flag) {
  auto& itp = detail::Interpreter::get();
  PyObject* res = PyObject_CallMethod(itp.plt, const_cast<char*>("grid"),
                                      const_cast<char*>("O"), flag ? Py_True : Py_False);
  if (!res) detail::throw_if_pyerr("plt.grid()");
  Py_DECREF(res);
}

inline void set_aspect_equal() {
  auto& itp = detail::Interpreter::get();
  PyObject* ax = PyObject_CallMethod(itp.plt, const_cast<char*>("gca"), nullptr);
  if (!ax) detail::throw_if_pyerr("plt.gca()");
  PyObject* res = PyObject_CallMethod(ax, const_cast<char*>("set_aspect"),
                                      const_cast<char*>("s"), "equal");
  Py_DECREF(ax);
  if (!res) detail::throw_if_pyerr("ax.set_aspect()");
  Py_DECREF(res);
}

template <typename Numeric>
inline bool plot(const std::vector<Numeric>& x, const std::vector<Numeric>& y,
                 const std::string& format = "") {
  if (x.size() != y.size()) throw std::runtime_error("plot(): x/y size mismatch");
  auto& itp = detail::Interpreter::get();

  PyObject* xlist = detail::py_list_from_vector(x);
  PyObject* ylist = detail::py_list_from_vector(y);

  PyObject* res = nullptr;
  if (format.empty()) {
    res = PyObject_CallMethod(itp.plt, const_cast<char*>("plot"),
                              const_cast<char*>("OO"), xlist, ylist);
  } else {
    res = PyObject_CallMethod(itp.plt, const_cast<char*>("plot"),
                              const_cast<char*>("OOs"), xlist, ylist, format.c_str());
  }

  Py_DECREF(xlist);
  Py_DECREF(ylist);
  if (!res) detail::throw_if_pyerr("plt.plot()");
  Py_DECREF(res);
  return true;
}

template <typename Numeric>
inline bool plot(const std::vector<Numeric>& x, const std::vector<Numeric>& y,
                 const std::map<std::string, std::string>& keywords) {
  if (x.size() != y.size()) throw std::runtime_error("plot(): x/y size mismatch");
  auto& itp = detail::Interpreter::get();

  PyObject* xlist = detail::py_list_from_vector(x);
  PyObject* ylist = detail::py_list_from_vector(y);
  PyObject* kwargs = detail::kwargs_from_map(keywords);

  PyObject* args = PyTuple_New(2);
  PyTuple_SetItem(args, 0, xlist);  // steals
  PyTuple_SetItem(args, 1, ylist);  // steals

  PyObject* res = PyObject_Call(PyObject_GetAttrString(itp.plt, "plot"), args, kwargs);
  Py_DECREF(args);
  Py_DECREF(kwargs);
  if (!res) detail::throw_if_pyerr("plt.plot(kwargs)");
  Py_DECREF(res);
  return true;
}

template <typename Numeric>
inline bool named_plot(const std::string& label, const std::vector<Numeric>& x,
                       const std::vector<Numeric>& y,
                       const std::string& format = "") {
  std::map<std::string, std::string> kw;
  kw["label"] = label;
  if (!format.empty()) {
    kw["linestyle"] = format;
  }
  return plot(x, y, kw);
}

template <typename Numeric>
inline bool scatter(const std::vector<Numeric>& x, const std::vector<Numeric>& y,
                    const double s = 20.0,
                    const std::map<std::string, std::string>& keywords = {}) {
  if (x.size() != y.size()) throw std::runtime_error("scatter(): x/y size mismatch");
  auto& itp = detail::Interpreter::get();

  PyObject* xlist = detail::py_list_from_vector(x);
  PyObject* ylist = detail::py_list_from_vector(y);
  PyObject* kwargs = detail::kwargs_from_map(keywords);
  PyDict_SetItemString(kwargs, "s", PyFloat_FromDouble(s));

  PyObject* args = PyTuple_New(2);
  PyTuple_SetItem(args, 0, xlist);  // steals
  PyTuple_SetItem(args, 1, ylist);  // steals

  PyObject* res = PyObject_Call(PyObject_GetAttrString(itp.plt, "scatter"), args, kwargs);
  Py_DECREF(args);
  Py_DECREF(kwargs);
  if (!res) detail::throw_if_pyerr("plt.scatter()");
  Py_DECREF(res);
  return true;
}

inline void save(const std::string& filename, const int dpi = 0) {
  auto& itp = detail::Interpreter::get();
  if (dpi > 0) {
    PyObject* kwargs = PyDict_New();
    PyDict_SetItemString(kwargs, "dpi", PyLong_FromLong(dpi));
    PyObject* res = PyObject_CallMethod(itp.plt, const_cast<char*>("savefig"),
                                        const_cast<char*>("s"), filename.c_str(), kwargs);
    Py_DECREF(kwargs);
    if (!res) detail::throw_if_pyerr("plt.savefig(dpi)");
    Py_DECREF(res);
  } else {
    PyObject* res = PyObject_CallMethod(itp.plt, const_cast<char*>("savefig"),
                                        const_cast<char*>("s"), filename.c_str());
    if (!res) detail::throw_if_pyerr("plt.savefig()");
    Py_DECREF(res);
  }
}

inline void legend() {
  auto& itp = detail::Interpreter::get();
  PyObject* res = PyObject_CallMethod(itp.plt, const_cast<char*>("legend"), nullptr);
  if (!res) detail::throw_if_pyerr("plt.legend()");
  Py_DECREF(res);
}

inline void legend(const std::string& loc) {
  auto& itp = detail::Interpreter::get();
  PyObject* kwargs = PyDict_New();
  PyObject* py_loc = PyUnicode_FromString(loc.c_str());
  PyDict_SetItemString(kwargs, "loc", py_loc);
  Py_DECREF(py_loc);

  PyObject* legend_func = PyObject_GetAttrString(itp.plt, "legend");
  PyObject* args = PyTuple_New(0);
  PyObject* res = PyObject_Call(legend_func, args, kwargs);
  Py_DECREF(legend_func);
  Py_DECREF(args);
  Py_DECREF(kwargs);
  if (!res) detail::throw_if_pyerr("plt.legend(loc=...)");
  Py_DECREF(res);
}

inline void text(double x, double y, const std::string& s) {
  auto& itp = detail::Interpreter::get();
  PyObject* res = PyObject_CallMethod(itp.plt, const_cast<char*>("text"),
                                      const_cast<char*>("dds"), x, y, s.c_str());
  if (!res) detail::throw_if_pyerr("plt.text()");
  Py_DECREF(res);
}

inline void pause(double interval) {
  auto& itp = detail::Interpreter::get();
  PyObject* res = PyObject_CallMethod(itp.plt, const_cast<char*>("pause"),
                                      const_cast<char*>("d"), interval);
  if (!res) detail::throw_if_pyerr("plt.pause()");
  Py_DECREF(res);
}

inline void axis(const std::string& mode) {
  if (mode == "equal") {
    set_aspect_equal();
    return;
  }
  auto& itp = detail::Interpreter::get();
  PyObject* res = PyObject_CallMethod(itp.plt, const_cast<char*>("axis"),
                                      const_cast<char*>("s"), mode.c_str());
  if (!res) detail::throw_if_pyerr("plt.axis()");
  Py_DECREF(res);
}

inline void show(const bool block = true) {
  auto& itp = detail::Interpreter::get();
  if (block) {
    PyObject* res = PyObject_CallMethod(itp.plt, const_cast<char*>("show"), nullptr);
    if (!res) detail::throw_if_pyerr("plt.show()");
    Py_DECREF(res);
  } else {
    PyObject* kwargs = PyDict_New();
    PyDict_SetItemString(kwargs, "block", Py_False);
    PyObject* show_func = PyObject_GetAttrString(itp.plt, "show");
    PyObject* args = PyTuple_New(0);
    PyObject* res = PyObject_Call(show_func, args, kwargs);
    Py_DECREF(show_func);
    Py_DECREF(args);
    Py_DECREF(kwargs);
    if (!res) detail::throw_if_pyerr("plt.show(block=False)");
    Py_DECREF(res);
  }
}

}  // namespace matplotlibcpp
