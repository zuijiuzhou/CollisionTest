#include "collision_manager_sap.h"

#define TMPL_PREFIX_ template <typename S>

/** @brief hyperbrain project */
namespace hyperbrain {
/** @brief robots */
namespace robot {
TMPL_PREFIX_
CollisionManagerSaP<S>::CollisionManagerSaP() {
    elist[0] = nullptr;
    elist[1] = nullptr;
    elist[2] = nullptr;

    optimal_axis = 0;
}

TMPL_PREFIX_
CollisionManagerSaP<S>::~CollisionManagerSaP() {
    clear();
}

TMPL_PREFIX_
void CollisionManagerSaP<S>::registerObjects(const std::vector<fcl::CollisionObject<S>*>& other_objs) {
    if (other_objs.empty()) return;

    if (size() > 0)
        CollisionManagerSaP<S>::registerObjects(other_objs);
    else {
        std::vector<EndPoint*> endpoints(2 * other_objs.size());

        for (size_t i = 0; i < other_objs.size(); ++i) {
            SaPAABB* sapaabb = new SaPAABB();
            sapaabb->obj = other_objs[i];
            sapaabb->lo = new EndPoint();
            sapaabb->hi = new EndPoint();
            sapaabb->cached = other_objs[i]->getAABB();
            endpoints[2 * i] = sapaabb->lo;
            endpoints[2 * i + 1] = sapaabb->hi;
            sapaabb->lo->minmax = 0;
            sapaabb->hi->minmax = 1;
            sapaabb->lo->aabb = sapaabb;
            sapaabb->hi->aabb = sapaabb;
            AABB_arr.push_back(sapaabb);
            obj_aabb_map[other_objs[i]] = sapaabb;
        }


        S scale[3];
        for (size_t coord = 0; coord < 3; ++coord) {
            std::sort(endpoints.begin(), endpoints.end(), std::bind(std::less<S>(), std::bind(static_cast<S (EndPoint::*)(size_t) const>(&EndPoint::getVal), std::placeholders::_1, coord), std::bind(static_cast<S (EndPoint::*)(size_t) const>(&EndPoint::getVal), std::placeholders::_2, coord)));

            endpoints[0]->prev[coord] = nullptr;
            endpoints[0]->next[coord] = endpoints[1];
            for (size_t i = 1; i < endpoints.size() - 1; ++i) {
                endpoints[i]->prev[coord] = endpoints[i - 1];
                endpoints[i]->next[coord] = endpoints[i + 1];
            }
            endpoints[endpoints.size() - 1]->prev[coord] = endpoints[endpoints.size() - 2];
            endpoints[endpoints.size() - 1]->next[coord] = nullptr;

            elist[coord] = endpoints[0];

            scale[coord] = endpoints.back()->aabb->cached.max_[coord] - endpoints[0]->aabb->cached.min_[coord];
        }

        int axis = 0;
        if (scale[axis] < scale[1]) axis = 1;
        if (scale[axis] < scale[2]) axis = 2;

        EndPoint* pos = elist[axis];

        while (pos != nullptr) {
            EndPoint* pos_next = nullptr;
            SaPAABB* aabb = pos->aabb;
            EndPoint* pos_it = pos->next[axis];

            while (pos_it != nullptr) {
                if (pos_it->aabb == aabb) {
                    if (pos_next == nullptr) pos_next = pos_it;
                    break;
                }

                if (pos_it->minmax == 0) {
                    if (pos_next == nullptr) pos_next = pos_it;
                    if (pos_it->aabb->cached.overlap(aabb->cached))
                        overlap_pairs.emplace_back(pos_it->aabb->obj, aabb->obj);
                }
                pos_it = pos_it->next[axis];
            }

            pos = pos_next;
        }
    }

    updateVelist();
}

TMPL_PREFIX_
void CollisionManagerSaP<S>::registerObject(fcl::CollisionObject<S>* obj) {
    SaPAABB* curr = new SaPAABB;
    curr->cached = obj->getAABB();
    curr->obj = obj;
    curr->lo = new EndPoint;
    curr->lo->minmax = 0;
    curr->lo->aabb = curr;

    curr->hi = new EndPoint;
    curr->hi->minmax = 1;
    curr->hi->aabb = curr;

    for (int coord = 0; coord < 3; ++coord) {
        EndPoint* current = elist[coord];

        // first insert the lo end point
        if (current == nullptr) // empty list
        {
            elist[coord] = curr->lo;
            curr->lo->prev[coord] = curr->lo->next[coord] = nullptr;
        }
        else // otherwise, find the correct location in the list and insert
        {
            EndPoint* curr_lo = curr->lo;
            S curr_lo_val = curr_lo->getVal()[coord];
            while ((current->getVal()[coord] < curr_lo_val) && (current->next[coord] != nullptr))
                current = current->next[coord];

            if (current->getVal()[coord] >= curr_lo_val) {
                curr_lo->prev[coord] = current->prev[coord];
                curr_lo->next[coord] = current;
                if (current->prev[coord] == nullptr)
                    elist[coord] = curr_lo;
                else
                    current->prev[coord]->next[coord] = curr_lo;

                current->prev[coord] = curr_lo;
            }
            else {
                curr_lo->prev[coord] = current;
                curr_lo->next[coord] = nullptr;
                current->next[coord] = curr_lo;
            }
        }

        // now insert hi end point
        current = curr->lo;

        EndPoint* curr_hi = curr->hi;
        S curr_hi_val = curr_hi->getVal()[coord];

        if (coord == 0) {
            while ((current->getVal()[coord] < curr_hi_val) && (current->next[coord] != nullptr)) {
                if (current != curr->lo)
                    if (current->aabb->cached.overlap(curr->cached))
                        overlap_pairs.emplace_back(current->aabb->obj, obj);

                current = current->next[coord];
            }
        }
        else {
            while ((current->getVal()[coord] < curr_hi_val) && (current->next[coord] != nullptr))
                current = current->next[coord];
        }

        if (current->getVal()[coord] >= curr_hi_val) {
            curr_hi->prev[coord] = current->prev[coord];
            curr_hi->next[coord] = current;
            if (current->prev[coord] == nullptr)
                elist[coord] = curr_hi;
            else
                current->prev[coord]->next[coord] = curr_hi;

            current->prev[coord] = curr_hi;
        }
        else {
            curr_hi->prev[coord] = current;
            curr_hi->next[coord] = nullptr;
            current->next[coord] = curr_hi;
        }
    }

    AABB_arr.push_back(curr);

    obj_aabb_map[obj] = curr;

    updateVelist();
}

TMPL_PREFIX_
void CollisionManagerSaP<S>::unregisterObject(fcl::CollisionObject<S>* obj) {
    auto it = AABB_arr.begin();
    for (auto end = AABB_arr.end(); it != end; ++it) {
        if ((*it)->obj == obj)
            break;
    }

    AABB_arr.erase(it);
    obj_aabb_map.erase(obj);

    if (it == AABB_arr.end())
        return;

    SaPAABB* curr = *it;
    *it = nullptr;

    for (int coord = 0; coord < 3; ++coord) {
        // first delete the lo endpoint of the interval.
        if (curr->lo->prev[coord] == nullptr)
            elist[coord] = curr->lo->next[coord];
        else
            curr->lo->prev[coord]->next[coord] = curr->lo->next[coord];

        curr->lo->next[coord]->prev[coord] = curr->lo->prev[coord];

        // then, delete the "hi" endpoint.
        if (curr->hi->prev[coord] == nullptr)
            elist[coord] = curr->hi->next[coord];
        else
            curr->hi->prev[coord]->next[coord] = curr->hi->next[coord];

        if (curr->hi->next[coord] != nullptr)
            curr->hi->next[coord]->prev[coord] = curr->hi->prev[coord];
    }

    delete curr->lo;
    delete curr->hi;
    delete curr;

    overlap_pairs.remove_if(isUnregistered(obj));
}

TMPL_PREFIX_
void CollisionManagerSaP<S>::setup() {
    if (size() == 0) return;

    S scale[3];
    scale[0] = (velist[0].back())->getVal(0) - velist[0][0]->getVal(0);
    scale[1] = (velist[1].back())->getVal(1) - velist[1][0]->getVal(1);
    ;
    scale[2] = (velist[2].back())->getVal(2) - velist[2][0]->getVal(2);
    size_t axis = 0;
    if (scale[axis] < scale[1]) axis = 1;
    if (scale[axis] < scale[2]) axis = 2;
    optimal_axis = axis;
}

TMPL_PREFIX_
void CollisionManagerSaP<S>::update() {
    for (auto it = AABB_arr.cbegin(), end = AABB_arr.cend(); it != end; ++it) {
        update_(*it);
    }

    updateVelist();

    setup();
}

TMPL_PREFIX_
void CollisionManagerSaP<S>::update(fcl::CollisionObject<S>* updated_obj) {
    update_(obj_aabb_map[updated_obj]);

    updateVelist();

    setup();
}

TMPL_PREFIX_
void CollisionManagerSaP<S>::update(const std::vector<fcl::CollisionObject<S>*>& updated_objs) {
    for (size_t i = 0; i < updated_objs.size(); ++i)
        update_(obj_aabb_map[updated_objs[i]]);

    updateVelist();

    setup();
}

TMPL_PREFIX_
void CollisionManagerSaP<S>::clear() {
    for (auto it = AABB_arr.begin(), end = AABB_arr.end(); it != end; ++it) {
        delete (*it)->hi;
        delete (*it)->lo;
        delete *it;
        *it = nullptr;
    }

    AABB_arr.clear();
    overlap_pairs.clear();

    elist[0] = nullptr;
    elist[1] = nullptr;
    elist[2] = nullptr;

    velist[0].clear();
    velist[1].clear();
    velist[2].clear();

    obj_aabb_map.clear();
}

TMPL_PREFIX_
void CollisionManagerSaP<S>::getObjects(std::vector<fcl::CollisionObject<S>*>& objs) const {
    objs.resize(AABB_arr.size());
    int i = 0;
    for (auto it = AABB_arr.cbegin(), end = AABB_arr.cend(); it != end; ++it, ++i) {
        objs[i] = (*it)->obj;
    }
}

TMPL_PREFIX_
bool CollisionManagerSaP<S>::empty() const {
    return AABB_arr.size();
}

TMPL_PREFIX_
size_t CollisionManagerSaP<S>::size() const {
    return AABB_arr.size();
}

TMPL_PREFIX_
void CollisionManagerSaP<S>::collide(CollisionData<S>& cdata, CollisionCallBack<S> callback) const {
    if (size() == 0) return;

    for (auto it = overlap_pairs.cbegin(), end = overlap_pairs.cend(); it != end; ++it) {
        fcl::CollisionObject<S>* obj1 = it->obj1;
        fcl::CollisionObject<S>* obj2 = it->obj2;

        if (callback(obj1, obj2, cdata))
            break;
    }
    cdata.done = true;
}

TMPL_PREFIX_
void CollisionManagerSaP<S>::collide(CollisionManager<S>* other, CollisionData<S>& cdata, CollisionCallBack<S> callback) const {
    auto* other_manager = dynamic_cast<CollisionManagerSaP<S>*>(other);

    if ((size() == 0) || (other_manager->size() == 0)) return;

    if (this == other_manager) {
        collide(cdata, callback);
        return;
    }

    if (this->size() < other_manager->size()) {
        for (auto it = AABB_arr.cbegin(); it != AABB_arr.cend(); ++it) {
            if (other_manager->collide_((*it)->obj, cdata, callback))
                break;
        }
    }
    else {
        for (auto it = other_manager->AABB_arr.cbegin(), end = other_manager->AABB_arr.cend(); it != end; ++it) {
            if (collide_((*it)->obj, cdata, callback))
                break;
        }
    }
    cdata.done = true;
}

TMPL_PREFIX_
void CollisionManagerSaP<S>::withInDistance(fcl::CollisionObject<S>* obj, WithInDistanceData<S>& cdata, WithInDistanceCallBack<S> callback) const {
}

TMPL_PREFIX_
void CollisionManagerSaP<S>::withInDistance(WithInDistanceData<S>& cdata, WithInDistanceCallBack<S> callback) const {
    if (size() == 0) return;

    this->enable_tested_set_ = true;
    this->tested_set.clear();

    for (auto it = AABB_arr.cbegin(), end = AABB_arr.cend(); it != end; ++it) {
        if (withInDistance_((*it)->obj, cdata, callback))
            break;
    }

    this->enable_tested_set_ = false;
    this->tested_set.clear();
    cdata.done = true;
}

TMPL_PREFIX_
void CollisionManagerSaP<S>::withInDistance(CollisionManager<S>* other, WithInDistanceData<S>& cdata, WithInDistanceCallBack<S> callback) const {
    auto other_manager = static_cast<CollisionManagerSaP<S>*>(other);

    if ((size() == 0) || (other_manager->size() == 0)) return;

    if (this == other_manager) {
        withInDistance(cdata, callback);
        return;
    }

    if (this->size() < other_manager->size()) {
        for (auto it = AABB_arr.cbegin(), end = AABB_arr.cend(); it != end; ++it) {
            if (other_manager->withInDistance_((*it)->obj, cdata, callback))
                return;
        }
    }
    else {
        for (auto it = other_manager->AABB_arr.cbegin(), end = other_manager->AABB_arr.cend(); it != end; ++it) {
            if (withInDistance_((*it)->obj, cdata, callback))
                return;
        }
    }
}

//==============================================================================
TMPL_PREFIX_
bool CollisionManagerSaP<S>::collide_(fcl::CollisionObject<S>* obj, CollisionData<S>& cdata, CollisionCallBack<S> callback) const {
    using namespace fcl;

    size_t axis = optimal_axis;
    const AABB<S>& obj_aabb = obj->getAABB();

    S min_val = obj_aabb.min_[axis];
    //  S max_val = obj_aabb.max_[axis];

    EndPoint dummy;
    SaPAABB dummy_aabb;
    dummy_aabb.cached = obj_aabb;
    dummy.minmax = 1;
    dummy.aabb = &dummy_aabb;

    // compute stop_pos by binary search, this is cheaper than check it in while iteration linearly
    const auto res_it = std::upper_bound(velist[axis].begin(), velist[axis].end(), &dummy, std::bind(std::less<S>(), std::bind(static_cast<S (EndPoint::*)(size_t) const>(&EndPoint::getVal), std::placeholders::_1, axis), std::bind(static_cast<S (EndPoint::*)(size_t) const>(&EndPoint::getVal), std::placeholders::_2, axis)));

    EndPoint* end_pos = nullptr;
    if (res_it != velist[axis].end())
        end_pos = *res_it;

    EndPoint* pos = elist[axis];

    while (pos != end_pos) {
        if (pos->aabb->obj != obj) {
            if (this->isExcludedBetween(obj, pos->aabb->obj)) {
                pos = pos->next[axis];
                continue;
            }
            if ((pos->minmax == 0) && (pos->aabb->hi->getVal(axis) >= min_val)) {
                if (pos->aabb->cached.overlap(obj->getAABB()))
                    if (callback(obj, pos->aabb->obj, cdata))
                        return true;
            }
        }
        pos = pos->next[axis];
    }

    return false;
}

TMPL_PREFIX_
bool CollisionManagerSaP<S>::withInDistance_(fcl::CollisionObject<S>* obj, WithInDistanceData<S>& cdata, WithInDistanceCallBack<S> callback) const {
    using namespace fcl;

    S min_dist = std::numeric_limits<S>::max();

    Vector3<S> delta = (obj->getAABB().max_ - obj->getAABB().min_) * 0.5;
    AABB<S> aabb = obj->getAABB();

    if (min_dist < std::numeric_limits<S>::max()) {
        Vector3<S> min_dist_delta(min_dist, min_dist, min_dist);
        aabb.expand(min_dist_delta);
    }

    size_t axis = optimal_axis;

    int status = 1;
    S old_min_distance;

    EndPoint* start_pos = elist[axis];

    while (1) {
        old_min_distance = min_dist;
        S min_val = aabb.min_[axis];
        //    S max_val = aabb.max_[axis];

        EndPoint dummy;
        SaPAABB dummy_aabb;
        dummy_aabb.cached = aabb;
        dummy.minmax = 1;
        dummy.aabb = &dummy_aabb;

        const auto res_it = std::upper_bound(
            velist[axis].begin(),
            velist[axis].end(),
            &dummy,
            std::bind(std::less<S>(),
                      std::bind(static_cast<S (EndPoint::*)(size_t) const>(&EndPoint::getVal), std::placeholders::_1, axis),
                      std::bind(static_cast<S (EndPoint::*)(size_t) const>(&EndPoint::getVal), std::placeholders::_2, axis)));

        EndPoint* end_pos = nullptr;
        if (res_it != velist[axis].end())
            end_pos = *res_it;

        EndPoint* pos = start_pos;

        while (pos != end_pos) {
            // can change to pos->aabb->hi->getVal(axis) >= min_val - min_dist, and then update start_pos to end_pos.
            // but this seems slower.
            if ((pos->minmax == 0) && (pos->aabb->hi->getVal(axis) >= min_val)) {
                CollisionObject<S>* curr_obj = pos->aabb->obj;
                if (curr_obj != obj) {
                    if (this->isExcludedBetween(obj, curr_obj)) {
                        pos = pos->next[axis];
                        continue;
                    }
                    if (!this->enable_tested_set_) {
                        if (pos->aabb->cached.distance(obj->getAABB()) < min_dist) {

                            cdata.request.safety_dist = this->getSafetyDistanceBetween(curr_obj, obj);

                            if (callback(curr_obj, obj, cdata))
                                return true;
                        }
                    }
                    else {
                        if (!this->inTestedSet(curr_obj, obj)) {
                            if (pos->aabb->cached.distance(obj->getAABB()) < min_dist) {

                                cdata.request.safety_dist = this->getSafetyDistanceBetween(curr_obj, obj);

                                if (callback(curr_obj, obj, cdata))
                                    return true;
                            }

                            this->insertTestedSet(curr_obj, obj);
                        }
                    }
                }
            }

            pos = pos->next[axis];
        }

        if (status == 1) {
            if (old_min_distance < std::numeric_limits<S>::max())
                break;
            else {
                if (min_dist < old_min_distance) {
                    Vector3<S> min_dist_delta(min_dist, min_dist, min_dist);
                    // an AABB centered as core and is of half-dimension delta
                    aabb = AABB<S>(obj->getAABB(), min_dist_delta);
                    status = 0;
                }
                else {
                    if (aabb.equal(obj->getAABB()))
                        aabb.expand(delta);
                    else
                        aabb.expand(obj->getAABB(), 2.0);
                }
            }
        }
        else if (status == 0)
            break;
    }

    return false;
}

TMPL_PREFIX_
void CollisionManagerSaP<S>::addToOverlapPairs(const SaPPair& p) {
    bool repeated = false;
    for (auto it = overlap_pairs.begin(), end = overlap_pairs.end(); it != end; ++it) {
        if (*it == p) {
            repeated = true;
            break;
        }
    }

    if (!repeated)
        overlap_pairs.push_back(p);
}

TMPL_PREFIX_
void CollisionManagerSaP<S>::removeFromOverlapPairs(const SaPPair& p) {
    for (auto it = overlap_pairs.begin(), end = overlap_pairs.end();
         it != end;
         ++it) {
        if (*it == p) {
            overlap_pairs.erase(it);
            break;
        }
    }
}

TMPL_PREFIX_
void CollisionManagerSaP<S>::update_(SaPAABB* updated_aabb) {
    using namespace fcl;

    if (updated_aabb->cached.equal(updated_aabb->obj->getAABB()))
        return;

    SaPAABB* current = updated_aabb;

    Vector3<S> new_min = current->obj->getAABB().min_;
    Vector3<S> new_max = current->obj->getAABB().max_;

    SaPAABB dummy;
    dummy.cached = current->obj->getAABB();

    for (int coord = 0; coord < 3; ++coord) {
        int direction; // -1 reverse, 0 nochange, 1 forward
        EndPoint* temp;

        if (current->lo->getVal(coord) > new_min[coord])
            direction = -1;
        else if (current->lo->getVal(coord) < new_min[coord])
            direction = 1;
        else
            direction = 0;

        if (direction == -1) {
            // first update the "lo" endpoint of the interval
            if (current->lo->prev[coord] != nullptr) {
                temp = current->lo;
                while ((temp != nullptr) && (temp->getVal(coord) > new_min[coord])) {
                    if (temp->minmax == 1)
                        if (temp->aabb->cached.overlap(dummy.cached))
                            addToOverlapPairs(SaPPair(temp->aabb->obj, current->obj));
                    temp = temp->prev[coord];
                }

                if (temp == nullptr) {
                    current->lo->prev[coord]->next[coord] = current->lo->next[coord];
                    current->lo->next[coord]->prev[coord] = current->lo->prev[coord];
                    current->lo->prev[coord] = nullptr;
                    current->lo->next[coord] = elist[coord];
                    elist[coord]->prev[coord] = current->lo;
                    elist[coord] = current->lo;
                }
                else {
                    current->lo->prev[coord]->next[coord] = current->lo->next[coord];
                    current->lo->next[coord]->prev[coord] = current->lo->prev[coord];
                    current->lo->prev[coord] = temp;
                    current->lo->next[coord] = temp->next[coord];
                    temp->next[coord]->prev[coord] = current->lo;
                    temp->next[coord] = current->lo;
                }
            }

            current->lo->getVal(coord) = new_min[coord];

            // update hi end point
            temp = current->hi;
            while (temp->getVal(coord) > new_max[coord]) {
                if ((temp->minmax == 0) && (temp->aabb->cached.overlap(current->cached)))
                    removeFromOverlapPairs(SaPPair(temp->aabb->obj, current->obj));
                temp = temp->prev[coord];
            }

            current->hi->prev[coord]->next[coord] = current->hi->next[coord];
            if (current->hi->next[coord] != nullptr)
                current->hi->next[coord]->prev[coord] = current->hi->prev[coord];
            current->hi->prev[coord] = temp;
            current->hi->next[coord] = temp->next[coord];
            if (temp->next[coord] != nullptr)
                temp->next[coord]->prev[coord] = current->hi;
            temp->next[coord] = current->hi;

            current->hi->getVal(coord) = new_max[coord];
        }
        else if (direction == 1) {
            // here, we first update the "hi" endpoint.
            if (current->hi->next[coord] != nullptr) {
                temp = current->hi;
                while ((temp->next[coord] != nullptr) && (temp->getVal(coord) < new_max[coord])) {
                    if (temp->minmax == 0)
                        if (temp->aabb->cached.overlap(dummy.cached))
                            addToOverlapPairs(SaPPair(temp->aabb->obj, current->obj));
                    temp = temp->next[coord];
                }

                if (temp->getVal(coord) < new_max[coord]) {
                    current->hi->prev[coord]->next[coord] = current->hi->next[coord];
                    current->hi->next[coord]->prev[coord] = current->hi->prev[coord];
                    current->hi->prev[coord] = temp;
                    current->hi->next[coord] = nullptr;
                    temp->next[coord] = current->hi;
                }
                else {
                    current->hi->prev[coord]->next[coord] = current->hi->next[coord];
                    current->hi->next[coord]->prev[coord] = current->hi->prev[coord];
                    current->hi->prev[coord] = temp->prev[coord];
                    current->hi->next[coord] = temp;
                    temp->prev[coord]->next[coord] = current->hi;
                    temp->prev[coord] = current->hi;
                }
            }

            current->hi->getVal(coord) = new_max[coord];

            // then, update the "lo" endpoint of the interval.
            temp = current->lo;

            while (temp->getVal(coord) < new_min[coord]) {
                if ((temp->minmax == 1) && (temp->aabb->cached.overlap(current->cached)))
                    removeFromOverlapPairs(SaPPair(temp->aabb->obj, current->obj));
                temp = temp->next[coord];
            }

            if (current->lo->prev[coord] != nullptr)
                current->lo->prev[coord]->next[coord] = current->lo->next[coord];
            else
                elist[coord] = current->lo->next[coord];
            current->lo->next[coord]->prev[coord] = current->lo->prev[coord];
            current->lo->prev[coord] = temp->prev[coord];
            current->lo->next[coord] = temp;
            if (temp->prev[coord] != nullptr)
                temp->prev[coord]->next[coord] = current->lo;
            else
                elist[coord] = current->lo;
            temp->prev[coord] = current->lo;
            current->lo->getVal(coord) = new_min[coord];
        }
    }
}

TMPL_PREFIX_
void CollisionManagerSaP<S>::updateVelist() {
    for (int coord = 0; coord < 3; ++coord) {
        velist[coord].resize(size() * 2);
        EndPoint* current = elist[coord];
        size_t id = 0;
        while (current) {
            velist[coord][id] = current;
            current = current->next[coord];
            id++;
        }
    }
}

//==============================================================================
TMPL_PREFIX_
const fcl::Vector3<S>& CollisionManagerSaP<S>::EndPoint::getVal() const {
    if (minmax)
        return aabb->cached.max_;
    else
        return aabb->cached.min_;
}

//==============================================================================
TMPL_PREFIX_
fcl::Vector3<S>& CollisionManagerSaP<S>::EndPoint::getVal() {
    if (minmax)
        return aabb->cached.max_;
    else
        return aabb->cached.min_;
}

//==============================================================================
TMPL_PREFIX_
S CollisionManagerSaP<S>::EndPoint::getVal(size_t i) const {
    if (minmax)
        return aabb->cached.max_[i];
    else
        return aabb->cached.min_[i];
}

//==============================================================================
TMPL_PREFIX_
S& CollisionManagerSaP<S>::EndPoint::getVal(size_t i) {
    if (minmax)
        return aabb->cached.max_[i];
    else
        return aabb->cached.min_[i];
}

//==============================================================================
TMPL_PREFIX_
CollisionManagerSaP<S>::SaPPair::SaPPair(fcl::CollisionObject<S>* a, fcl::CollisionObject<S>* b) {
    if (a < b) {
        obj1 = a;
        obj2 = b;
    }
    else {
        obj1 = b;
        obj2 = a;
    }
}

//==============================================================================
TMPL_PREFIX_
bool CollisionManagerSaP<S>::SaPPair::operator==(const typename CollisionManagerSaP<S>::SaPPair& other) const {
    return ((obj1 == other.obj1) && (obj2 == other.obj2));
}

//==============================================================================
TMPL_PREFIX_
CollisionManagerSaP<S>::isUnregistered::isUnregistered(fcl::CollisionObject<S>* obj_)
  : obj(obj_) {
}

//==============================================================================
TMPL_PREFIX_
bool CollisionManagerSaP<S>::isUnregistered::operator()(const SaPPair& pair) const {
    return (pair.obj1 == obj) || (pair.obj2 == obj);
}

//==============================================================================
TMPL_PREFIX_
CollisionManagerSaP<S>::isNotValidPair::isNotValidPair(fcl::CollisionObject<S>* obj1_, fcl::CollisionObject<S>* obj2_)
  : obj1(obj1_)
  , obj2(obj2_) {
    // Do nothing
}

//==============================================================================
TMPL_PREFIX_
bool CollisionManagerSaP<S>::isNotValidPair::operator()(const SaPPair& pair) {
    return (pair.obj1 == obj1) && (pair.obj2 == obj2);
}

template class CollisionManagerSaP<float>;
template class CollisionManagerSaP<double>;

#undef TMPL_PREFIX_
} // namespace robot
} // namespace hyperbrain