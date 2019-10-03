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
            v-if="objectKey"
            class="float-left"
          >
            {{ el[objectKey] }}
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
export default {
  name: 'interactive-list',
  props: {
    list: {
      type: Array,
      required: true,
    },
    objectKey: {
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
