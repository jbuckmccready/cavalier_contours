#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/**
 * Opaque type that wraps a [StaticAABB2DIndex].
 *
 * Note the internal member is only public for composing in other Rust libraries wanting to use the
 * FFI opaque type as part of their FFI API.
 */
typedef struct cavc_aabbindex cavc_aabbindex;

/**
 * Opaque type that wraps a [Polyline].
 *
 * Note the internal member is only public for composing in other Rust libraries wanting to use the
 * FFI opaque type as part of their FFI API.
 */
typedef struct cavc_pline cavc_pline;

/**
 * Opaque type that represents a list of [cavc_pline].
 *
 * Note the internal member is only public for composing in other Rust libraries wanting to use the
 * FFI opaque type as part of their FFI API.
 */
typedef struct cavc_plinelist cavc_plinelist;

/**
 * Opaque type that wraps a [Shape].
 *
 * Note the internal member is only public for composing in other Rust libraries wanting to use the
 * FFI opaque type as part of their FFI API.
 */
typedef struct cavc_shape cavc_shape;

/**
 * FFI representation of [PlineOffsetOptions].
 */
typedef struct cavc_pline_parallel_offset_o {
  const struct cavc_aabbindex *aabb_index;
  double pos_equal_eps;
  double slice_join_eps;
  double offset_dist_eps;
  uint8_t handle_self_intersects;
} cavc_pline_parallel_offset_o;

/**
 * FFI representation of [PlineBooleanOptions].
 */
typedef struct cavc_pline_boolean_o {
  const struct cavc_aabbindex *pline1_aabb_index;
  double pos_equal_eps;
} cavc_pline_boolean_o;

/**
 * Represents a polyline vertex holding x, y, and bulge.
 */
typedef struct cavc_vertex {
  double x;
  double y;
  double bulge;
} cavc_vertex;

/**
 * FFI representation of [ShapeOffsetOptions].
 */
typedef struct cavc_shape_offset_o {
  double pos_equal_eps;
  double offset_dist_eps;
  double slice_join_eps;
} cavc_shape_offset_o;

/**
 * Write default option values to a [cavc_pline_parallel_offset_o].
 *
 * ## Specific Error Codes
 * * 1 = `options` is null.
 *
 * # Safety
 *
 * `options` must point to a valid place in memory to be written.
 */
int32_t cavc_pline_parallel_offset_o_init(struct cavc_pline_parallel_offset_o *options);

/**
 * Write default option values to a [cavc_pline_boolean_o].
 *
 * ## Specific Error Codes
 * * 1 = `options` is null.
 *
 * # Safety
 *
 * `options` must point to a valid place in memory to be written.
 */
int32_t cavc_pline_boolean_o_init(struct cavc_pline_boolean_o *options);

/**
 * Create a new polyline object.
 *
 * `vertexes` is an array of [cavc_vertex] to create the polyline with (may be null if `n_vertexes`
 * is 0).
 * `n_vertexes` contains the number of vertexes in the array.
 * `is_closed` sets the polyline to be closed if non-zero.
 * `pline` is an out parameter to hold the created polyline.
 *
 * # Safety
 *
 * `vertexes` may be null if `n_vertexes` is 0 or must point to a valid contiguous buffer of
 * [cavc_vertex] with length of at least `n_vertexes`.
 * `pline` must point to a valid place in memory to be written.
 */
int32_t cavc_pline_create(const struct cavc_vertex *vertexes,
                          uint32_t n_vertexes,
                          uint8_t is_closed,
                          const struct cavc_pline **pline);

/**
 * Free an existing [cavc_pline] object.
 *
 * Nothing happens if `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not already been freed.
 */
void cavc_pline_f(struct cavc_pline *pline);

/**
 * Set the userdata values of a pline
 *
 * 'userdata_values' is a user-provided array of u64 that is stored with a pline and preserved across offset calls.
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 * `userdata_values` must point to a valid location to read from.
 */
int32_t cavc_pline_set_userdata_values(struct cavc_pline *pline,
                                       const uint64_t *userdata_values,
                                       uint32_t count);

/**
 * Get the userdata value count of a polyline.
 *
 * `count` used as out parameter to hold the vertex count.
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 * `count` must point to a valid place in memory to be written.
 */
int32_t cavc_pline_get_userdata_count(const struct cavc_pline *pline, uint32_t *count);

/**
 * Get the userdata values of a pline
 *
 * 'userdata_values' is a user-provided C array of u64 that is stored with a pline and preserved across offset calls.
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 * `userdata_values` must point to a buffer that is large enough to hold all the userdata values or a buffer
 * overrun will happen.
 */
int32_t cavc_pline_get_userdata_values(const struct cavc_pline *pline,
                                       uint64_t *userdata_values);

/**
 * Reserve space for an `additional` number of vertexes in the [cavc_pline].
 *
 * This function is used to avoid allocations when adding vertexes to the [cavc_pline].
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 */
int32_t cavc_pline_reserve(struct cavc_pline *pline, uint32_t additional);

/**
 * Clones the polyline.
 *
 * `pline` is the polyline to be cloned.
 * `cloned` is used as an out parameter to hold the new polyline from cloning.
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 * `cloned` must point to a valid place in memory to be written.
 */
int32_t cavc_pline_clone(const struct cavc_pline *pline, const struct cavc_pline **cloned);

/**
 * Get whether the polyline is closed or not.
 *
 * `is_closed` is used as an out parameter to hold the whether `pline` is closed (non-zero) or not
 * (zero).
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 * `is_closed` must point to a valid place in memory to be written.
 */
int32_t cavc_pline_get_is_closed(const struct cavc_pline *pline, uint8_t *is_closed);

/**
 * Set whether the polyline is closed or not.
 *
 * If `is_closed` is non-zero then `pline` is set to be closed, otherwise it is set to be open.
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 */
int32_t cavc_pline_set_is_closed(struct cavc_pline *pline, uint8_t is_closed);

/**
 * Get the vertex count of a polyline.
 *
 * `count` used as out parameter to hold the vertex count.
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 * `count` must point to a valid place in memory to be written.
 */
int32_t cavc_pline_get_vertex_count(const struct cavc_pline *pline, uint32_t *count);

/**
 * Fills the buffer given with the vertex data of a polyline.
 *
 * You must use [cavc_pline_get_vertex_count] to ensure the buffer given has adequate length
 * to be filled with all vertexes!
 *
 * `vertex_data` must point to a buffer that can be filled with all `pline` vertexes.
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 * `vertex_data` must point to a buffer that is large enough to hold all the vertexes or a buffer
 * overrun will happen.
 */
int32_t cavc_pline_get_vertex_data(const struct cavc_pline *pline, struct cavc_vertex *vertex_data);

/**
 * Sets all of the vertexes of a polyline.
 *
 * `vertex_data` is an array of vertexes to use for the polyline.
 * `n_vertexes` must specify the number of vertexes to be read from the
 * `vertex_data` array.
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 * `vertex_data` must be a valid pointer to a buffer of at least `n_vertexes` of [cavc_vertex].
 */
int32_t cavc_pline_set_vertex_data(struct cavc_pline *pline,
                                   const struct cavc_vertex *vertex_data,
                                   uint32_t n_vertexes);

/**
 * Clears all of the vertexes of a polyline.
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 */
int32_t cavc_pline_clear(struct cavc_pline *pline);

/**
 * Add a vertex to a polyline `pline` with `x`, `y`, and `bulge`.
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 */
int32_t cavc_pline_add(struct cavc_pline *pline, double x, double y, double bulge);

/**
 * Get a polyline vertex at a given index position.
 *
 * `position` is is the index to get the vertex at.
 * `vertex` used as out parameter to hold the vertex retrieved.
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 * * 2 = `position` is out of bounds for the `pline` given.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 * `vertex` must point to a valid place in memory to be written.
 */
int32_t cavc_pline_get_vertex(const struct cavc_pline *pline,
                              uint32_t position,
                              struct cavc_vertex *vertex);

/**
 * Set a polyline vertex at a given index position.
 *
 * `position` is is the index to set the vertex at.
 * `vertex` is the data to be set.
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 * * 2 = `position` is out of bounds for the `pline` given.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 */
int32_t cavc_pline_set_vertex(struct cavc_pline *pline,
                              uint32_t position,
                              struct cavc_vertex vertex);

/**
 * Remove a vertex from a polyline at an index position.
 *
 * `position` is the index of the vertex to be removed from the polyline.
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 * * 2 = `position` is out of bounds for the `pline` given.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 */
int32_t cavc_pline_remove(struct cavc_pline *pline, uint32_t position);

/**
 * Wraps [PlineSource::path_length].
 *
 * `path_length` is used as the out parameter to hold the computed path length.
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 * `path_length` must point to a valid place in memory to be written.
 */
int32_t cavc_pline_eval_path_length(const struct cavc_pline *pline, double *path_length);

/**
 * Wraps [PlineSource::area].
 *
 * `area` is used as the out parameter to hold the computed area.
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 * `area` must point to a valid place in memory to be written.
 */
int32_t cavc_pline_eval_area(const struct cavc_pline *pline, double *area);

/**
 * Wraps [PlineSource::winding_number].
 *
 * `winding_number` is used as the out parameter to hold the computed winding number.
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 * `winding_number` must point to a valid place in memory to be written.
 */
int32_t cavc_pline_eval_wn(const struct cavc_pline *pline,
                           double x,
                           double y,
                           int32_t *winding_number);

/**
 * Wraps [PlineSourceMut::invert_direction_mut].
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 */
int32_t cavc_pline_invert_direction(struct cavc_pline *pline);

/**
 * Wraps [PlineSourceMut::scale_mut].
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 */
int32_t cavc_pline_scale(struct cavc_pline *pline, double scale_factor);

/**
 * Wraps [PlineSourceMut::translate_mut].
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 */
int32_t cavc_pline_translate(struct cavc_pline *pline, double x_offset, double y_offset);

/**
 * Wraps [PlineSource::remove_repeat_pos] but modifies in place rather than returning a result.
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 */
int32_t cavc_pline_remove_repeat_pos(struct cavc_pline *pline, double pos_equal_eps);

/**
 * Wraps [PlineSource::remove_redundant] but modifies in place rather than returning a result.
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 */
int32_t cavc_pline_remove_redundant(struct cavc_pline *pline, double pos_equal_eps);

/**
 * Wraps [PlineSource::extents].
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 * * 2 = `pline` vertex count is less than 2.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 * `min_x`, `min_y`, `max_x`, and `max_y` must all point to a valid places in memory to be written.
 */
int32_t cavc_pline_eval_extents(const struct cavc_pline *pline,
                                double *min_x,
                                double *min_y,
                                double *max_x,
                                double *max_y);

/**
 * Wraps [PlineSource::parallel_offset_opt].
 *
 * `options` is allowed to be null (default options will be used).
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 * `result` must point to a valid place in memory to be written.
 */
int32_t cavc_pline_parallel_offset(const struct cavc_pline *pline,
                                   double offset,
                                   const struct cavc_pline_parallel_offset_o *options,
                                   const struct cavc_plinelist **result);

/**
 * Wraps [PlineSource::boolean_opt].
 *
 * `options` is allowed to be null (default options will be used).
 *
 * Boolean operations are:
 * * 0 = [BooleanOp::Or]
 * * 1 = [BooleanOp::And]
 * * 2 = [BooleanOp::Not]
 * * 3 = [BooleanOp::Xor]
 *
 * ## Specific Error Codes
 * * 1 = `pline1` and/or `pline2` is null.
 * * 2 = `operation` is unrecognized (must be one of the values listed).
 *
 * # Safety
 *
 * `pline1` and `pline2` must each be null or a valid cavc_pline object that was created with
 * [cavc_pline_create] and has not been freed.
 * `pos_plines` and `neg_plines` must both point to different valid places in memory to be written.
 */
int32_t cavc_pline_boolean(const struct cavc_pline *pline1,
                           const struct cavc_pline *pline2,
                           uint32_t operation,
                           const struct cavc_pline_boolean_o *options,
                           const struct cavc_plinelist **pos_plines,
                           const struct cavc_plinelist **neg_plines);

/**
 * Wraps [PlineSource::create_approx_aabb_index].
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 * `aabbindex` must point to a valid place in memory to be written.
 */
int32_t cavc_pline_create_approx_aabbindex(const struct cavc_pline *pline,
                                           const struct cavc_aabbindex **aabbindex);

/**
 * Wraps [PlineSource::create_aabb_index].
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 * `aabbindex` must point to a valid place in memory to be written.
 */
int32_t cavc_pline_create_aabbindex(const struct cavc_pline *pline,
                                    const struct cavc_aabbindex **aabbindex);

/**
 * Free an existing [cavc_aabbindex] object.
 *
 * Nothing happens if `aabbindex` is null.
 *
 * # Safety
 *
 * `aabbindex` must be null or a valid [cavc_aabbindex] object.
 */
void cavc_aabbindex_f(struct cavc_aabbindex *aabbindex);

/**
 * Wraps the [`StaticAABB2DIndex::bounds`] method (gets total extents of the aabb index). Writes
 * NaNs if the index is empty.
 *
 * ## Specific Error Codes
 * * 1 = `aabbindex` is null.
 *
 * # Safety
 *
 * `aabbindex` must be null or a valid [cavc_aabbindex] object.
 * `min_x`, `min_y`, `max_x`, and `max_y` must all point to a valid places in memory to be written.
 */
int32_t cavc_aabbindex_get_extents(const struct cavc_aabbindex *aabbindex,
                                   double *min_x,
                                   double *min_y,
                                   double *max_x,
                                   double *max_y);

/**
 * Create a new [cavc_plinelist] object.
 *
 * `capacity` is the number of plines to pre-allocate space for. May be zero.
 * `plinelist` is an out parameter to hold the created shape.
 *
 * # Safety
 *
 * `plinelist` must point to a valid place in memory to be written.
 */
int32_t cavc_plinelist_create(uintptr_t capacity, struct cavc_plinelist **plinelist);

/**
 * Free an existing [cavc_plinelist] object and all [cavc_pline] owned by it.
 *
 * Nothing happens if `plinelist` is null.
 *
 * # Safety
 *
 * `plinelist` must be null or a valid [cavc_plinelist] object.
 */
void cavc_plinelist_f(struct cavc_plinelist *plinelist);

/**
 * Get the number of polylines inside a [cavc_plinelist].
 *
 * `count` used as out parameter to hold the polyline count.
 *
 * ## Specific Error Codes
 * * 1 = `plinelist` is null.
 *
 * # Safety
 *
 * `plinelist` must be null or a valid [cavc_plinelist] object.
 * `count` must point to a valid place in memory to be written.
 */
int32_t cavc_plinelist_get_count(const struct cavc_plinelist *plinelist, uint32_t *count);

/**
 * Get a polyline at the given index position in the [cavc_plinelist].
 *
 * `pline` used as out parameter to hold the polyline pointer. NOTE: This does not release
 * ownership of the [cavc_pline] from the [cavc_plinelist], to do that use [cavc_plinelist_pop] or
 * [cavc_plinelist_take].
 *
 * ## Specific Error Codes
 * * 1 = `plinelist` is null.
 * * 2 = `position` out of range for the [cavc_plinelist].
 *
 * # Safety
 *
 * `plinelist` must be null or a valid [cavc_plinelist] object.
 * `pline` must point to a valid place in memory to be written.
 */
int32_t cavc_plinelist_get_pline(const struct cavc_plinelist *plinelist,
                                 uint32_t position,
                                 const struct cavc_pline **pline);

/**
 * Append a [cavc_pline] to the end of a [cavc_plinelist].
 *
 * `plinelist` is the [cavc_plinelist] to append to.
 * `pline` is the [cavc_pline] to be appended.
 *
 * ## Specific Error Codes
 * * 1 = `plinelist` is null.
 * * 2 = `pline` is null.
 *
 * # Safety
 *
 * `plinelist` must be a valid [cavc_plinelist] object.
 * `pline` must be a valid [cavc_pline] object.
 */
int32_t cavc_plinelist_push(struct cavc_plinelist *plinelist, struct cavc_pline *pline);

/**
 * Efficiently release and return the last [cavc_pline] from a [cavc_plinelist].
 *
 * `pline` used as out parameter to hold the polyline pointer released from the [cavc_plinelist].
 * NOTE: The caller now must call [cavc_pline_f] at some point to free the released [cavc_pline].
 *
 * ## Specific Error Codes
 * * 1 = `plinelist` is null.
 * * 2 = `plinelist` is empty.
 *
 * # Safety
 *
 * `plinelist` must be null or a valid [cavc_plinelist] object.
 * `pline` must point to a valid place in memory to be written.
 */
int32_t cavc_plinelist_pop(struct cavc_plinelist *plinelist, const struct cavc_pline **pline);

/**
 * Release and return a [cavc_pline] from a [cavc_plinelist] at a given index position.
 *
 * `pline` used as out parameter to hold the polyline pointer released from the [cavc_plinelist].
 * NOTE: The caller now must call [cavc_pline_f] at some point to free the released [cavc_pline].
 *
 * ## Specific Error Codes
 * * 1 = `plinelist` is null.
 * * 2 = `position` out of range for the [cavc_plinelist].
 *
 * # Safety
 *
 * `plinelist` must be null or a valid [cavc_plinelist] object.
 * `pline` must point to a valid place in memory to be written.
 */
int32_t cavc_plinelist_take(struct cavc_plinelist *plinelist,
                            uint32_t position,
                            const struct cavc_pline **pline);

/**
 * Write default option values to a [cavc_shape_offset_o].
 *
 * ## Specific Error Codes
 * * 1 = `options` is null.
 *
 * # Safety
 *
 * `options` must point to a valid place in memory to be written.
 */
int32_t cavc_shape_offset_o_init(struct cavc_shape_offset_o *options);

/**
 * Create a new [cavc_shape] object.
 *
 * `plinelist` is a [cavc_plinelist] containing the [cavc_pline] paths to create the shape from.
 * `shape` is an out parameter to hold the created shape.
 *
 * ## Specific Error Codes
 * * 1 = `plinelist` is null.
 *
 * # Safety
 *
 * `shape` must point to a valid place in memory to be written.
 */
int32_t cavc_shape_create(const struct cavc_plinelist *plinelist, struct cavc_shape **shape);

/**
 * Free an existing [cavc_shape] object.
 *
 * Nothing happens if `shape` is null.
 *
 * # Safety
 *
 * `shape` must be null or a valid cavc_shape object that was created with [cavc_shape_create] and
 * has not already been freed.
 */
void cavc_shape_f(struct cavc_shape *shape);

/**
 * Wraps [Shape::parallel_offset].
 *
 * `options` is allowed to be null (default options will be used).
 *
 * ## Specific Error Codes
 * * 1 = `shape` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 * `result` must point to a valid place in memory to be written.
 */
int32_t cavc_shape_parallel_offset(const struct cavc_shape *shape,
                                   double offset,
                                   const struct cavc_shape_offset_o *options,
                                   struct cavc_shape **result);

/**
 * Get the count of counter-clockwise polylines in a shape.
 *
 * `count` used as out parameter to hold the vertex count.
 *
 * ## Specific Error Codes
 * * 1 = `shape` is null.
 *
 * # Safety
 *
 * `shape` must be null or a valid cavc_shape object that was created with [cavc_pline_create] and
 * has not been freed.
 * `count` must point to a valid place in memory to be written.
 */
int32_t cavc_shape_get_ccw_count(const struct cavc_shape *shape, uint32_t *count);

/**
 * Get the vertex count of a specific counter-clockwise polyline in a shape.
 *
 * `count` used as out parameter to hold the vertex count.
 *
 * ## Specific Error Codes
 * * 1 = `shape` is null.
 * * 2 = `polyline_index` is beyond the bounds of the count of the shape's ccw polylines
 *
 * # Safety
 *
 * `shape` must be null or a valid cavc_shape object that was created with [cavc_pline_create] and
 * has not been freed.
 * `count` must point to a valid place in memory to be written.
 */
int32_t cavc_shape_get_ccw_polyline_count(const struct cavc_shape *shape,
                                          uint32_t polyline_index,
                                          uint32_t *count);

/**
 * Get whether a specific counter-clockwise polyline in a shape is closed.
 *
 * `is_closed` is used as an out parameter to hold the whether the polyline is closed (non-zero) or not
 * (zero).
 *
 * ## Specific Error Codes
 * * 1 = `shape` is null.
 * * 2 = `polyline_index` is beyond the bounds of the count of the shape's ccw polylines
 *
 * # Safety
 *
 * `shape` must be null or a valid cavc_shape object that was created with [cavc_pline_create] and
 * has not been freed.
 * `is_closed` must point to a valid place in memory to be written.
 */
int32_t cavc_shape_get_ccw_polyline_is_closed(const struct cavc_shape *shape,
                                              uint32_t polyline_index,
                                              uint8_t *is_closed);

/**
 * Fills the buffer given with the vertex data of a ccw polyline in a shape.
 *
 * You must use [cavc_shape_get_ccw_polyline_count] to ensure the buffer given has adequate length
 * to be filled with all vertexes!
 *
 * `vertex_data` must point to a buffer that can be filled with all `pline` vertexes.
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 * * 2 = `polyline_index` is beyond the bounds of the count of the shape's ccw polylines
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 * `vertex_data` must point to a buffer that is large enough to hold all the vertexes or a buffer
 * overrun will happen.
 */
int32_t cavc_shape_get_ccw_polyline_vertex_data(const struct cavc_shape *shape,
                                                uint32_t polyline_index,
                                                struct cavc_vertex *vertex_data);

/**
 * Set the userdata values of a CCW polyline in a shape
 *
 * 'userdata_values' is a user-provided array of u64 that is stored with a pline and preserved across offset calls.
 *
 * ## Specific Error Codes
 * * 1 = `shape` is null.
 * * 2 = `polyline_index` is beyond the bounds of the count of the shape's ccw polylines
 *
 * # Safety
 *
 * `shape` must be null or a valid cavc_shape object that was created with [cavc_shape_create] and
 * has not been freed.
 * `userdata_values` must point to a valid location to read from.
 */
int32_t cavc_shape_set_ccw_pline_userdata_values(struct cavc_shape *shape,
                                                 uint32_t polyline_index,
                                                 const uint64_t *userdata_values,
                                                 uint32_t count);

/**
 * Get the userdata value count of a CCW polyline in a shape.
 *
 * `count` used as out parameter to hold the vertex count.
 *
 * ## Specific Error Codes
 * * 1 = `shape` is null.
 * * 2 = `polyline_index` is beyond the bounds of the count of the shape's ccw polylines
 *
 * # Safety
 *
 * `shape` must be null or a valid cavc_shape object that was created with [cavc_shape_create] and
 * has not been freed.
 * `count` must point to a valid place in memory to be written.
 */
int32_t cavc_shape_get_ccw_pline_userdata_count(const struct cavc_shape *shape,
                                                uint32_t polyline_index,
                                                uint32_t *count);

/**
 * Get the userdata values of a CCW pline in a shape
 *
 * 'userdata_values' is a user-provided C array of u64 that is stored with a pline and preserved across offset calls.
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 * `userdata_values` must point to a buffer that is large enough to hold all the userdata values or a buffer
 * overrun will happen.
 */
int32_t cavc_shape_get_ccw_pline_userdata_values(const struct cavc_shape *shape,
                                                 uint32_t polyline_index,
                                                 uint64_t *userdata_values);

/**
 * Get the count of clockwise polylines in a shape.
 *
 * `count` used as out parameter to hold the vertex count.
 *
 * ## Specific Error Codes
 * * 1 = `shape` is null.
 *
 * # Safety
 *
 * `shape` must be null or a valid cavc_shape object that was created with [cavc_pline_create] and
 * has not been freed.
 * `count` must point to a valid place in memory to be written.
 */
int32_t cavc_shape_get_cw_count(const struct cavc_shape *shape, uint32_t *count);

/**
 * Get the vertex count of a specific clockwise polyline in a shape.
 *
 * `count` used as out parameter to hold the vertex count.
 *
 * ## Specific Error Codes
 * * 1 = `shape` is null.
 * * 2 = `polyline_index` is beyond the bounds of the count of the shape's cw polylines
 *
 * # Safety
 *
 * `shape` must be null or a valid cavc_shape object that was created with [cavc_pline_create] and
 * has not been freed.
 * `count` must point to a valid place in memory to be written.
 */
int32_t cavc_shape_get_cw_polyline_count(const struct cavc_shape *shape,
                                         uint32_t polyline_index,
                                         uint32_t *count);

/**
 * Get whether a specific clockwise polyline in a shape is closed.
 *
 * `is_closed` is used as an out parameter to hold the whether the polyline is closed (non-zero) or not
 * (zero).
 *
 * ## Specific Error Codes
 * * 1 = `shape` is null.
 * * 2 = `polyline_index` is beyond the bounds of the count of the shape's ccw polylines
 *
 * # Safety
 *
 * `shape` must be null or a valid cavc_shape object that was created with [cavc_pline_create] and
 * has not been freed.
 * `is_closed` must point to a valid place in memory to be written.
 */
int32_t cavc_shape_get_cw_polyline_is_closed(const struct cavc_shape *shape,
                                             uint32_t polyline_index,
                                             uint8_t *is_closed);

/**
 * Fills the buffer given with the vertex data of a cw polyline in a shape.
 *
 * You must use [cavc_shape_get_cw_polyline_count] to ensure the buffer given has adequate length
 * to be filled with all vertexes!
 *
 * `vertex_data` must point to a buffer that can be filled with all `pline` vertexes.
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 * * 2 = `polyline_index` is beyond the bounds of the count of the shape's cw polylines
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 * `vertex_data` must point to a buffer that is large enough to hold all the vertexes or a buffer
 * overrun will happen.
 */
int32_t cavc_shape_get_cw_polyline_vertex_data(const struct cavc_shape *shape,
                                               uint32_t polyline_index,
                                               struct cavc_vertex *vertex_data);

/**
 * Set the userdata values of a CW polyline in a shape
 *
 * 'userdata_values' is a user-provided array of u64 that is stored with a pline and preserved across offset calls.
 *
 * ## Specific Error Codes
 * * 1 = `shape` is null.
 * * 2 = `polyline_index` is beyond the bounds of the count of the shape's cw polylines
 *
 * # Safety
 *
 * `shape` must be null or a valid cavc_shape object that was created with [cavc_shape_create] and
 * has not been freed.
 * `userdata_values` must point to a valid location to read from.
 */
int32_t cavc_shape_set_cw_pline_userdata_values(struct cavc_shape *shape,
                                                uint32_t polyline_index,
                                                const uint64_t *userdata_values,
                                                uint32_t count);

/**
 * Get the userdata value count of a CW polyline in a shape.
 *
 * `count` used as out parameter to hold the vertex count.
 *
 * ## Specific Error Codes
 * * 1 = `shape` is null.
 * * 2 = `polyline_index` is beyond the bounds of the count of the shape's cw polylines
 *
 * # Safety
 *
 * `shape` must be null or a valid cavc_shape object that was created with [cavc_shape_create] and
 * has not been freed.
 * `count` must point to a valid place in memory to be written.
 */
int32_t cavc_shape_get_cw_pline_userdata_count(const struct cavc_shape *shape,
                                               uint32_t polyline_index,
                                               uint32_t *count);

/**
 * Get the userdata values of a CW pline in a shape
 *
 * 'userdata_values' is a user-provided C array of u64 that is stored with a pline and preserved across offset calls.
 *
 * ## Specific Error Codes
 * * 1 = `pline` is null.
 *
 * # Safety
 *
 * `pline` must be null or a valid cavc_pline object that was created with [cavc_pline_create] and
 * has not been freed.
 * `userdata_values` must point to a buffer that is large enough to hold all the userdata values or a buffer
 * overrun will happen.
 */
int32_t cavc_shape_get_cw_pline_userdata_values(const struct cavc_shape *shape,
                                                uint32_t polyline_index,
                                                uint64_t *userdata_values);
