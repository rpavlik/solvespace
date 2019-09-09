//-----------------------------------------------------------------------------
// Data structures used frequently in the program, various kinds of vectors
// (of real numbers, not symbolic algebra stuff) and our templated lists.
//
// Copyright 2008-2013 Jonathan Westhues.
//-----------------------------------------------------------------------------
#ifndef SOLVESPACE_DSC_H
#define SOLVESPACE_DSC_H

#include "solvespace.h"

#include <type_traits>

/// Trait indicating which types are handle types and should get the associated operators.
/// Specialize for each handle type and inherit from std::true_type.
template<typename T>
struct IsHandleOracle : std::false_type {};

// Equality-compare any two instances of a handle type.
template<typename T>
static inline typename std::enable_if<IsHandleOracle<T>::value, bool>::type
operator==(T const &lhs, T const &rhs) {
    return lhs.v == rhs.v;
}

// Inequality-compare any two instances of a handle type.
template<typename T>
static inline typename std::enable_if<IsHandleOracle<T>::value, bool>::type
operator!=(T const &lhs, T const &rhs) {
    return !(lhs == rhs);
}

// Less-than-compare any two instances of a handle type.
template<typename T>
static inline typename std::enable_if<IsHandleOracle<T>::value, bool>::type
operator<(T const &lhs, T const &rhs) {
    return lhs.v < rhs.v;
}

class Vector;
class Vector4;
class Point2d;
class hEntity;
class hParam;

class Quaternion {
public:
    // a + (vx)*i + (vy)*j + (vz)*k
    double w, vx, vy, vz;

    static const Quaternion IDENTITY;

    static Quaternion From(double w, double vx, double vy, double vz);
    static Quaternion From(hParam w, hParam vx, hParam vy, hParam vz);
    static Quaternion From(Vector u, Vector v);
    static Quaternion From(Vector axis, double dtheta);

    Quaternion Plus(Quaternion b) const;
    Quaternion Minus(Quaternion b) const;
    Quaternion ScaledBy(double s) const;
    double Magnitude() const;
    Quaternion WithMagnitude(double s) const;

    // Call a rotation matrix [ u' v' n' ]'; this returns the first and
    // second rows, where that matrix is generated by this quaternion
    Vector RotationU() const;
    Vector RotationV() const;
    Vector RotationN() const;
    Vector Rotate(Vector p) const;

    Quaternion ToThe(double p) const;
    Quaternion Inverse() const;
    Quaternion Times(Quaternion b) const;
    Quaternion Mirror() const;
};

class Vector {
public:
    double x, y, z;

    static Vector From(double x, double y, double z);
    static Vector From(hParam x, hParam y, hParam z);
    static Vector AtIntersectionOfPlanes(Vector n1, double d1,
                                         Vector n2, double d2);
    static Vector AtIntersectionOfLines(Vector a0, Vector a1,
                                        Vector b0, Vector b1,
                                        bool *skew,
                                        double *pa=NULL, double *pb=NULL);
    static Vector AtIntersectionOfPlaneAndLine(Vector n, double d,
                                               Vector p0, Vector p1,
                                               bool *parallel);
    static Vector AtIntersectionOfPlanes(Vector na, double da,
                                         Vector nb, double db,
                                         Vector nc, double dc, bool *parallel);
    static void ClosestPointBetweenLines(Vector pa, Vector da,
                                         Vector pb, Vector db,
                                         double *ta, double *tb);

    double Element(int i) const;
    bool Equals(Vector v, double tol=LENGTH_EPS) const;
    bool EqualsExactly(Vector v) const;
    Vector Plus(Vector b) const;
    Vector Minus(Vector b) const;
    Vector Negated() const;
    Vector Cross(Vector b) const;
    double DirectionCosineWith(Vector b) const;
    double Dot(Vector b) const;
    Vector Normal(int which) const;
    Vector RotatedAbout(Vector orig, Vector axis, double theta) const;
    Vector RotatedAbout(Vector axis, double theta) const;
    Vector DotInToCsys(Vector u, Vector v, Vector n) const;
    Vector ScaleOutOfCsys(Vector u, Vector v, Vector n) const;
    double DistanceToLine(Vector p0, Vector dp) const;
    double DistanceToPlane(Vector normal, Vector origin) const;
    bool OnLineSegment(Vector a, Vector b, double tol=LENGTH_EPS) const;
    Vector ClosestPointOnLine(Vector p0, Vector deltal) const;
    double Magnitude() const;
    double MagSquared() const;
    Vector WithMagnitude(double s) const;
    Vector ScaledBy(double s) const;
    Vector ProjectInto(hEntity wrkpl) const;
    Vector ProjectVectorInto(hEntity wrkpl) const;
    double DivProjected(Vector delta) const;
    Vector ClosestOrtho() const;
    void MakeMaxMin(Vector *maxv, Vector *minv) const;
    Vector ClampWithin(double minv, double maxv) const;
    static bool BoundingBoxesDisjoint(Vector amax, Vector amin,
                                      Vector bmax, Vector bmin);
    static bool BoundingBoxIntersectsLine(Vector amax, Vector amin,
                                          Vector p0, Vector p1, bool asSegment);
    bool OutsideAndNotOn(Vector maxv, Vector minv) const;
    Vector InPerspective(Vector u, Vector v, Vector n,
                         Vector origin, double cameraTan) const;
    Point2d Project2d(Vector u, Vector v) const;
    Point2d ProjectXy() const;
    Vector4 Project4d() const;
};

inline double Vector::Element(int i) const {
    switch (i) {
    case 0: return x;
    case 1: return y;
    case 2: return z;
    default: ssassert(false, "Unexpected vector element index");
    }
}

inline bool Vector::Equals(Vector v, double tol) const {
    // Quick axis-aligned tests before going further
    const Vector dv = this->Minus(v);
    if (fabs(dv.x) > tol) return false;
    if (fabs(dv.y) > tol) return false;
    if (fabs(dv.z) > tol) return false;

    return dv.MagSquared() < tol*tol;
}

struct VectorHash {
    size_t operator()(const Vector &v) const;
};

struct VectorPred {
    bool operator()(Vector a, Vector b) const;
};

class Vector4 {
public:
    double w, x, y, z;

    static Vector4 From(double w, double x, double y, double z);
    static Vector4 From(double w, Vector v3);
    static Vector4 Blend(Vector4 a, Vector4 b, double t);

    Vector4 Plus(Vector4 b) const;
    Vector4 Minus(Vector4 b) const;
    Vector4 ScaledBy(double s) const;
    Vector PerspectiveProject() const;
};

class Point2d {
public:
    double x, y;

    static Point2d From(double x, double y);
    static Point2d FromPolar(double r, double a);

    Point2d Plus(const Point2d &b) const;
    Point2d Minus(const Point2d &b) const;
    Point2d ScaledBy(double s) const;
    double DivProjected(Point2d delta) const;
    double Dot(Point2d p) const;
    double DistanceTo(const Point2d &p) const;
    double DistanceToLine(const Point2d &p0, const Point2d &dp, bool asSegment) const;
    double DistanceToLineSigned(const Point2d &p0, const Point2d &dp, bool asSegment) const;
    double Angle() const;
    double AngleTo(const Point2d &p) const;
    double Magnitude() const;
    double MagSquared() const;
    Point2d WithMagnitude(double v) const;
    Point2d Normal() const;
    bool Equals(Point2d v, double tol=LENGTH_EPS) const;
};
#undef SOLVESPACE_VERIFY_LIST_USAGE
#undef SOLVESPACE_VERIFY_LIST_USAGE_STRICT
#ifdef SOLVESPACE_VERIFY_LIST_USAGE
template<typename T>
class HasClear {
private:
    typedef std::true_type yes;
    typedef std::false_type no;
    // This overload only available if the type has a Clear() member function.
    // The comma in the expression makes it return bool anyway.
    template<typename U>
    static decltype(std::declval<U &>().Clear(), yes()) TestClear(int);
    // This overload is always available but lowest priority.
    template<typename U>
    static no TestClear(...);

public:
    static constexpr bool value = std::is_same<decltype(TestClear<T>(0)), yes>::value;
};
#endif // SOLVESPACE_VERIFY_LIST_USAGE

// Cleanup policy for SharedListStorage used by List:
// does no cleanup.
struct NullCleanup {
    template<class T>
    static void Cleanup(T & /* val */) {
    }
};

// Cleanup policy for SharedListStorage used by IdList:
// calls Clear() on each member before removal.
struct ClearCleanup {
    template<class T>
    static void Cleanup(T &val) {
        val.Clear();
    }
};

template<class T, typename Cleanup = NullCleanup>
struct SharedListStorage {
public:
    std::vector<T> vec;

private:
    int internalN = 0;

public:
    SharedListStorage()                          = default;
    SharedListStorage(const SharedListStorage &) = delete;
    SharedListStorage(SharedListStorage &&)      = delete;
    SharedListStorage &operator=(const SharedListStorage &) = delete;
    SharedListStorage &operator=(SharedListStorage &&) = delete;

    ~SharedListStorage() {
        Clear();
    }

    // Calls any cleanup on each element (depends on template type Cleanup),
    // then clears the list.
    void Clear() {
        for(auto &elt : vec) {
            Cleanup::Cleanup(elt);
        }
        vec.clear();
        UpdateSize();
    }

    // Returns a reference wrapper that can be used to provide a "size" or "n" member that acts like
    // a data member.
    std::reference_wrapper<const int> MakeN() const {
        return std::cref(internalN);
    }

    // Element access.
    T &operator[](std::size_t i) {
        return vec[i];
    }
    // Element access (const).
    const T &operator[](std::size_t i) const {
        return vec[i];
    }

    // Clears the list *without* calling cleanup on the elements.
    // Typically only used if ownership is being transfered to another list.
    void AbandonElements() {
        vec.clear();
        UpdateSize();
    }

    // Update the backing variable supporting the MakeN()-returned reference wrapper.
    // Must be called after each size-changing operation.
    // All size-changing member functions do this.
    void UpdateSize() {
        internalN = static_cast<int>(vec.size());
    }

    // Forwards to std::vector<>::emplace_back to construct new element in place
    template<typename... Args>
    void EmplaceBack(Args &&... a) {
        vec.emplace_back(std::forward<Args>(a)...);
        UpdateSize();
    }

    // Forwards to std::vector<>::push_back
    void PushBack(const T &val) {
        vec.push_back(val);
        UpdateSize();
    }

    // Uses std::vector::insert at .begin()
    // Theoretically up to O(n) because of moving things...
    void PushFront(const T &val) {
        vec.insert(vec.begin(), val);
        UpdateSize();
    }

    // If the capacity and size are the same, requests a re-allocation by the vector.
    void AllocForOneMore() {

        if(vec.capacity() == vec.size()) {
            // Let the built-in growth policy deal with this.
            vec.reserve(vec.size() + 1);
        }
    }

    // Reserves a minimum of the given number of additional elements.
    void ReserveMore(int howMuch) {
        vec.reserve(vec.size() + howMuch);
    }

    bool IsEmpty() const {
        return vec.empty();
    }

    // Removes elements with .tag = 1, performing cleanup if configured to do so.
    void RemoveTagged() {
        if(IsEmpty()) {
            return;
        }
        int dest = 0;
        int n    = vec.size();
        for(int src = 0; src < n; src++) {
            if(vec[src].tag) {
                // this item should be deleted
                Cleanup::Cleanup(vec[src]);
            } else {
                if(src != dest) {
                    vec[dest] = vec[src];
                }
                dest++;
            }
        }
        // resize will call the destructor
        vec.resize(dest);
        UpdateSize();
    }

    // Removes the last element, performing cleanup if configured to do so.
    void PopBack() {
        if(IsEmpty()) {
            return;
        }
        Cleanup::Cleanup(vec.back());
        vec.pop_back();
        UpdateSize();
    }

    // Removes the last cnt elements (starting with the last), performing cleanup if configured to
    // do so.
    void RemoveLast(int cnt) {
        int n = vec.size();
        ssassert(n >= cnt, "Removing more elements than the list contains");
        if(IsEmpty()) {
            return;
        }
        for(int i = 0; i < cnt; ++i) {
            PopBack();
        }
    }
};

// A simple list, where copies are by default shallow copies.
template<class T>
class List {
    T *elem            = nullptr;
    int elemsAllocated = 0;
#ifdef SOLVESPACE_VERIFY_LIST_USAGE_STRICT
    static_assert(HasClear<T>::value == false,
                  "The List class template does not Clear() before deleting!");
#endif // SOLVESPACE_VERIFY_LIST_USAGE
public:
    int  n = 0;


    bool IsEmpty() const { return n == 0; }

    void ReserveMore(int howMuch) {
        if(n + howMuch > elemsAllocated) {
            elemsAllocated = n + howMuch;
            T *newElem = (T *)MemAlloc((size_t)elemsAllocated*sizeof(T));
            for(int i = 0; i < n; i++) {
                new(&newElem[i]) T(std::move(elem[i]));
                elem[i].~T();
            }
            MemFree(elem);
            elem = newElem;
        }
    }

    void AllocForOneMore() {
        if(n >= elemsAllocated) {
            ReserveMore((elemsAllocated + 32)*2 - n);
        }
    }

    void Add(const T *t) {
        AllocForOneMore();
        new(&elem[n++]) T(*t);
    }

    void AddToBeginning(const T *t) {
        AllocForOneMore();
        new(&elem[n]) T();
        std::move_backward(elem, elem + 1, elem + n + 1);
        elem[0] = *t;
        n++;
    }

    T *First() {
        return IsEmpty() ? nullptr : &(elem[0]);
    }
    const T *First() const {
        return IsEmpty() ? nullptr : &(elem[0]);
    }

    T *Last() { return IsEmpty() ? nullptr : &(elem[n - 1]); }
    const T *Last() const { return IsEmpty() ? nullptr : &(elem[n - 1]); }

    T *NextAfter(T *prev) {
        if(IsEmpty() || !prev) return NULL;
        if(prev - First() == (n - 1)) return NULL;
        return prev + 1;
    }
    const T *NextAfter(const T *prev) const {
        if(IsEmpty() || !prev) return NULL;
        if(prev - First() == (n - 1)) return NULL;
        return prev + 1;
    }

    T &Get(size_t i) { return elem[i]; }
    T const &Get(size_t i) const { return elem[i]; }
    T &operator[](size_t i) { return Get(i); }
    T const &operator[](size_t i) const { return Get(i); }

    T *begin() { return IsEmpty() ? nullptr : &elem[0]; }
    T *end() { return IsEmpty() ? nullptr : &elem[n]; }
    const T *begin() const { return IsEmpty() ? nullptr : &elem[0]; }
    const T *end() const { return IsEmpty() ? nullptr : &elem[n]; }
    const T *cbegin() const { return begin(); }
    const T *cend() const { return end(); }

    void ClearTags() {
        for(auto & elt : *this) {
            elt.tag = 0;
        }
    }

    void Clear() {
        for(int i = 0; i < n; i++)
            elem[i].~T();
        if(elem) MemFree(elem);
        elem = NULL;
        n = elemsAllocated = 0;
    }

    void RemoveTagged() {
        auto newEnd = std::remove_if(this->begin(), this->end(), [](T &t) {
            if(t.tag) {
                return true;
            }
            return false;
        });
        auto oldEnd = this->end();
        n = newEnd - begin();
        if (newEnd != nullptr && oldEnd != nullptr) {
            while(newEnd != oldEnd) {
                newEnd->~T();
                ++newEnd;
            }
        }
        // and elemsAllocated is untouched, because we didn't resize
    }

    void RemoveLast(int cnt) {
        ssassert(n >= cnt, "Removing more elements than the list contains");
        for(int i = n - cnt; i < n; i++)
            elem[i].~T();
        n -= cnt;
        // and elemsAllocated is untouched, same as in RemoveTagged
    }

    void Reverse() {
        int i;
        for(i = 0; i < (n/2); i++) {
            swap(elem[i], elem[(n-1)-i]);
        }
    }
};

// Comparison functor used by IdList and related classes
template <class T, class H>
struct CompareId {
    bool operator()(T const& lhs, T const& rhs) const {
        return lhs.h.v < rhs.h.v;
    }
    bool operator()(T const& lhs, H rhs) const {
        return lhs.h.v < rhs.v;
    }
};


// A list, where each element has an integer identifier. The list is kept
// sorted by that identifier, and items can be looked up in log n time by
// id.
template<class T, class H>
class IdList {
#ifdef SOLVESPACE_VERIFY_LIST_USAGE
    static_assert(HasClear<T>::value,
                  "The IdList class template requires Clear(), which it calls before deleting!");
#endif // SOLVESPACE_VERIFY_LIST_USAGE
    std::shared_ptr<SharedListStorage<T, ClearCleanup>> storage;

    // DO NOT MODIFY this value - exists solely to provide a place to point the "n"
    // reference when we have no storage.
    int dummyN = 0;
    void Allocate() {
        storage = std::make_shared<SharedListStorage<T, ClearCleanup>>();
        n       = storage->MakeN();
    }
    void EnsureAllocated() {
        if(storage == nullptr) {
            Allocate();
        }
    }

    // Releases our reference to the list. If ours was the last remaining
    // reference to the list, it will be destroyed. Clear() will be called on all elements
    // before destruction.
    void Reset() {
        storage.reset();
        n = std::cref(dummyN);
    }

public:
    std::reference_wrapper<const int> n;

    using Compare = CompareId<T, H>;

    IdList() : n(std::cref(dummyN)) {
    }
    IdList(IdList const &) = default;
    IdList &operator=(IdList const &) = default;
    IdList(IdList &&)                 = default;
    IdList &operator=(IdList &&) = default;

    bool IsEmpty() const {
        if(storage == nullptr) {
            return true;
        }

        return n == 0;
    }

    void AllocForOneMore() {
        EnsureAllocated();
        storage->AllocForOneMore();
    }

    uint32_t MaximumId() {
        if(IsEmpty()) {
            return 0;
        }
        return Last()->h.v;
    }

    H AddAndAssignId(T *t) {
        t->h.v = (MaximumId() + 1);
        Add(t);

        return t->h;
    }

    T *LowerBound(T const &t) {
        if(IsEmpty()) {
            return nullptr;
        }
        auto it = std::lower_bound(begin(), end(), t, Compare());
        return it;
    }

    T *LowerBound(H const &h) {
        if(IsEmpty()) {
            return nullptr;
        }
        auto it = std::lower_bound(begin(), end(), h, Compare());
        return it;
    }

    int LowerBoundIndex(T const &t) {
        if(IsEmpty()) {
            return 0;
        }
        auto it  = LowerBound(t);
        auto idx = std::distance(begin(), it);
        auto i   = static_cast<int>(idx);
        return i;
    }
    void ReserveMore(int howMuch) {
        EnsureAllocated();
        storage->ReserveMore(howMuch);
    }

    void Add(T *t) {
        AllocForOneMore();

        // Look to see if we already have something with the same handle value.
        ssassert(FindByIdNoOops(t->h) == nullptr, "Handle isn't unique");

        // Copy-construct at the end of the list.
        storage->EmplaceBack(*t);
        // The item we just added is trivially sorted, so "merge"
        std::inplace_merge(begin(), end() - 1, end(), Compare());
    }

    T *FindById(H h) {
        T *t = FindByIdNoOops(h);
        ssassert(t != NULL, "Cannot find handle");
        return t;
    }

    int IndexOf(H h) {
        if(IsEmpty()) {
            return -1;
        }
        auto it  = LowerBound(h);
        auto idx = std::distance(begin(), it);
        if(idx < n) {
            return idx;
        }
        return -1;
    }

    T *FindByIdNoOops(H h) {
        if(IsEmpty()) {
            return nullptr;
        }
        auto it = LowerBound(h);
        if(it == nullptr || it == end()) {
            return nullptr;
        }
        if(it->h.v == h.v) {
            return it;
        }
        return nullptr;
    }

    T *First() {
        return (IsEmpty()) ? NULL : &((*storage)[0]);
    }
    T *Last() {
        return (IsEmpty()) ? NULL : &((*storage)[n - 1]);
    }
    T *NextAfter(T *prev) {
        if(IsEmpty() || !prev)
            return NULL;
        if(prev - First() == (n - 1))
            return NULL;
        return prev + 1;
    }

    T &Get(size_t i) {
        ssassert(i < (size_t)n, "Index out of range.");
        return (*this)[i];
    }
    T const &Get(size_t i) const {
        ssassert(i < (size_t)n, "Index out of range.");
        return (*this)[i];
    }
    T &operator[](size_t i) {
        return (*storage)[i];
    }
    T const &operator[](size_t i) const {
        return (*storage)[i];
    }

    T *begin() { return IsEmpty() ? nullptr : &((*storage)[0]); }
    T *end() { return IsEmpty() ? nullptr : begin() + n; }
    const T *begin() const { return IsEmpty() ? nullptr : &((*storage)[0]); }
    const T *end() const { return IsEmpty() ? nullptr : begin() + n; }
    const T *cbegin() const { return begin(); }
    const T *cend() const { return end(); }

    void ClearTags() {
        for(auto &elt : *this) { elt.tag = 0; }
    }

    void Tag(H h, int tag) {
        auto it = FindByIdNoOops(h);
        if(it != nullptr) {
            it->tag = tag;
        }
    }

    void RemoveTagged() {
        if(storage) {
            storage->RemoveTagged();
        }
    }
    void RemoveById(H h) {
        ClearTags();
        FindById(h)->tag = 1;
        RemoveTagged();
    }

    void MoveSelfInto(IdList<T, H> *l) {
        l->Clear();
        std::swap(l->storage, storage);
        std::swap(l->n, n);
        Reset();
    }

    void DeepCopyInto(IdList<T, H> *l) {
        l->Clear();
        l->ReserveMore(n);
        for(auto &elt : *this) {
            l->storage->EmplaceBack(elt);
        }
    }

    // Calls Clear() on all elements and removes them from the list,
    // then releases our reference to the list.
    // Other locations may still hold a reference to the list,
    // but the list is now empty.
    void Clear() {
        if(storage) {
            // Performing this separately, first, means that even if this isn't
            // the last reference (shallow copy) to this list,
            // it will still have all its contents removed.
            storage->Clear();
        }
        Reset();
    }
};

class BandedMatrix {
public:
    enum {
        MAX_UNKNOWNS   = 16,
        RIGHT_OF_DIAG  = 1,
        LEFT_OF_DIAG   = 2
    };

    double A[MAX_UNKNOWNS][MAX_UNKNOWNS];
    double B[MAX_UNKNOWNS];
    double X[MAX_UNKNOWNS];
    int n;

    void Solve();
};

#define RGBi(r, g, b) RgbaColor::From((r), (g), (b))
#define RGBf(r, g, b) RgbaColor::FromFloat((float)(r), (float)(g), (float)(b))

// Note: sizeof(class RgbaColor) should be exactly 4
//
class RgbaColor {
public:
    uint8_t red, green, blue, alpha;

    float redF()   const { return (float)red   / 255.0f; }
    float greenF() const { return (float)green / 255.0f; }
    float blueF()  const { return (float)blue  / 255.0f; }
    float alphaF() const { return (float)alpha / 255.0f; }

    bool IsEmpty() const { return alpha == 0; }

    bool Equals(RgbaColor c) const {
        return
            c.red   == red   &&
            c.green == green &&
            c.blue  == blue  &&
            c.alpha == alpha;
    }

    RgbaColor WithAlpha(uint8_t newAlpha) const {
        RgbaColor color = *this;
        color.alpha = newAlpha;
        return color;
    }

    uint32_t ToPackedIntBGRA() const {
        return
            blue |
            (uint32_t)(green << 8) |
            (uint32_t)(red << 16) |
            (uint32_t)((255 - alpha) << 24);
    }

    uint32_t ToPackedInt() const {
        return
            red |
            (uint32_t)(green << 8) |
            (uint32_t)(blue << 16) |
            (uint32_t)((255 - alpha) << 24);
    }

    uint32_t ToARGB32() const {
        return
            blue |
            (uint32_t)(green << 8) |
            (uint32_t)(red << 16) |
            (uint32_t)(alpha << 24);
    }

    static RgbaColor From(int r, int g, int b, int a = 255) {
        RgbaColor c;
        c.red   = (uint8_t)r;
        c.green = (uint8_t)g;
        c.blue  = (uint8_t)b;
        c.alpha = (uint8_t)a;
        return c;
    }

    static RgbaColor FromFloat(float r, float g, float b, float a = 1.0) {
        return From(
            (int)(255.1f * r),
            (int)(255.1f * g),
            (int)(255.1f * b),
            (int)(255.1f * a));
    }

    static RgbaColor FromPackedInt(uint32_t rgba) {
        return From(
            (int)((rgba)       & 0xff),
            (int)((rgba >> 8)  & 0xff),
            (int)((rgba >> 16) & 0xff),
            (int)(255 - ((rgba >> 24) & 0xff)));
    }

    static RgbaColor FromPackedIntBGRA(uint32_t bgra) {
        return From(
            (int)((bgra >> 16) & 0xff),
            (int)((bgra >> 8)  & 0xff),
            (int)((bgra)       & 0xff),
            (int)(255 - ((bgra >> 24) & 0xff)));
    }
};

struct RgbaColorCompare {
    bool operator()(RgbaColor a, RgbaColor b) const {
        return a.ToARGB32() < b.ToARGB32();
    }
};

class BBox {
public:
    Vector minp;
    Vector maxp;

    static BBox From(const Vector &p0, const Vector &p1);

    Vector GetOrigin() const;
    Vector GetExtents() const;

    void Include(const Vector &v, double r = 0.0);
    bool Overlaps(const BBox &b1) const;
    bool Contains(const Point2d &p, double r = 0.0) const;
};

#endif
