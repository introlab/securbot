<template>
  <div class="list-container">
    <div class="list-inner rounded-lg">
      <div class="custom-header">
        <slot name="header" />
      </div>
      <div class="list-content">
        <button
          v-for="(el, index) in list"
          :key="index"
          type="button"
          class="list-group-item-action list-group-item d-flex justify-content-between
            align-items-center item"
          @click="$emit('click', el)"
        >
          <div
            v-if="displayKey"
            class="float-left"
          >
            {{ el[displayKey] }}
          </div>
          <div
            v-else
            class="float-left"
          >
            {{ el }}
          </div>
          <div class="float-right">
            <slot
              name="tag"
              :item="el"
            />
          </div>
        </button>
      </div>
    </div>
  </div>
</template>

<script>
/**
 * An interactive list (list of button that emit events) component with 2 slots, a header/title
 * slot and a tag slot on each button (a html element that is added at the right of the button).
 *
 * NOTE: The displayed content of the buttons can be configured through the displayKey. The vue
 * html will call the displayKey as a property on each element, usefull if the elements of the the
 * list are objects:
 * ```javascript
 * {{ listElement[displayKey] }}
 * ```
 * @displayName Interactive List
 * @since 0.2.2
 * @version 1.0.0
 */
export default {
  name: 'interactive-list',
  props: {
    /**
     * The list of element to "transform" into buttons.
     */
    list: {
      type: Array,
      required: true,
    },
    /**
     * The property to display inside the button -> same as list[i].[displayName]
     */
    displayKey: {
      type: String,
      default: '',
    },
  },
};
</script>

<style scoped>
.item{
  min-width: 300px;
  min-height: 30px;
}
.list-container {
  min-width: 320px;
  margin: 0 auto 0 auto;
}
.list-inner {
  border: 1px solid lightgray;
}
.custom-header {
  height: auto;
  width: auto;
  padding: 1rem;
}
</style>
