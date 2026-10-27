/**
 * Extract the peripheral instance name from a dot-separated string.
 * On STM32 devices a windowed/paired comparator is bound as a sub-instance,
 * e.g. "COMP34.COMP4"; only the leaf ("COMP4") names the HAL handle
 * (mx_comp4_gethandle()). A plain instance without a dot (e.g. "COMP4") is
 * returned unchanged.
 *
 * For example: "COMP34.COMP4" -> "COMP4", "COMP4" -> "COMP4"
 *
 * @param {string} full_name Full peripheral path (e.g. "COMP34.COMP4")
 * @returns {string} Instance name (e.g. "COMP4"), or empty string on error
 */
function helper_rgbled_get_comp_instance_name(full_name) {
  try {
    console.info(
      `helper_rgbled_get_comp_instance_name: full_name=${JSON.stringify(full_name)}`
    );

    if (typeof full_name !== "string" || full_name.length === 0) {
      return "";
    }

    const parts = full_name.split(".");
    const instance_name = parts[parts.length - 1];

    return instance_name || "";
  } catch (e) {
    console.error(`helper_rgbled_get_comp_instance_name: ${e}`);
    return "";
  }
}

module.exports = {
  helper_rgbled_get_comp_instance_name
}
