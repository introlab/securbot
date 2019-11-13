<template>
  <div class="w-100 h-100 border rounded table-container shadow-sb">
    <!-- Table -->
    <table
      id="waypoints-table"
      class="h-100 table table-borderless table-striped table-hover border-left border-right m-0 stick-header"
      style="text-align: center"
    >
      <!-- Header -->
      <thead class="text-white">
        <tr>
          <th
            :style="{ width: columnWidth }"
          >
            #
          </th>
          <th
            v-for="header in headers"
            :key="header"
            :style="{ width: columnWidth }"
          >
            {{ header }}
          </th>
          <th
            v-if="removable"
            :style="{ width: columnWidth }"
          >
            Remove
          </th>
        </tr>
      </thead>
      <!-- Body -->
      <tbody>
        <tr
          v-for="(el, index) of list"
          :key="index"
          class="w-100"
        >
          <!-- # -->
          <td :style="{ width: columnWidth }">
            {{ (index+1).toFixed(0) }}
          </td>
          <!-- Dynamic -->
          <td
            v-for="(value, key) in el"
            :key="key"
            :style="{ width: columnWidth }"
          >
            <div v-if="typeof value === 'number'">
              {{ value.toFixed(1) }}
            </div>
            <div v-else>
              {{ value }}
            </div>
          </td>
          <!-- Remove button column -->
          <td
            v-if="removable"
            :style="{ width: columnWidth }"
          >
            <!-- Remove button -->
            <button
              :id="'removeBtn'+index"
              type="button"
              class="btn btn-danger p-0 m-0 border border-secondary h-100 w-100"
              @click="removeIndex(index)"
            >
              <font-awesome-icon icon="trash" />
            </button>
          </td>
        </tr>
      </tbody>
    </table>
  </div>
</template>

<script>
/**
 * To add/change
 *
 * - Add a select headers, aka make the header a list of object of the following format:
 * ```javascript
 * header: {
 *  name: String,
 *  prop: String,
 *  visual: Array<enums>
 *  style/class/css: Array<String>,
 * }
 * ```
 * - Add visuals style option for elements in row.
 *  - Add a badge option
 * - Change the removable prop for a prependCol/appendCol prop with multiple options
 *  - The prop should be given in an Array of string
 *    -(e.i: ['remove', 'select', 'load', 'propToLoad'])
 * - Add an option prop that enables some features/events (Given as an Array)
 *  - Add a hover row option
 *  - Add a select row option
 *  - Add a reorder row (by selecting) option
 */


/**
 * A generic vue table component that allows for dynamic columns and offers a remove row button as
 * its last row that emits an event when clicked.
 *
 * NOTE: In the future, the remove button and its event might be changed to become more generic and
 * make configurable (style).
 * @displayName Table
 * @since 0.2.2
 * @version 1.0.0
 */
export default {
  name: 'securbot-table',
  props: {
    /**
     * The desired headers.
     */
    headers: {
      type: Array,
      required: true,
    },
    /**
     * The list of object to enumerate by row.
     */
    list: {
      type: Array,
      required: true,
    },
    /**
     * Flag for the remove button column.
     */
    removable: {
      type: Boolean,
      default: false,
    },
  },
  data() {
    return {
      validProp: true,
      columnWidth: '20%',
    };
  },
  mounted() {
    const w = 100 / (this.headers.length + 2);
    this.columnWidth = `${w}%`;
  },
  methods: {
    /**
     * Gets called when a remove button is clicked.
     * @param {Number} index The row index of the clicked button.
     * @public
     */
    removeIndex(index) {
      console.log(index);
      /**
       * The remove row event.
       *
       * @type {Number} The index of the row to remove.
       */
      this.$emit('removeRow', index);
    },
  },
};
</script>

<style scoped>
.stick-header tbody{
  display: block;
  overflow: auto;
  height: 100%;
  width: 100%;
}
.stick-header thead tr {
  display: block;
}
th {
  background-color: #00A759;
}
.table-container {
  background-color: rgb(255, 255, 255);
}
</style>
