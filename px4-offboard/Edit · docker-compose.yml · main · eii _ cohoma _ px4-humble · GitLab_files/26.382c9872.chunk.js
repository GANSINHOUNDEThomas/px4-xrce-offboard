(this.webpackJsonp=this.webpackJsonp||[]).push([[26],{"3fen":function(e,t,s){"use strict";s.d(t,"a",(function(){return d})),s.d(t,"b",(function(){return n})),s.d(t,"c",(function(){return c}));var l=s("6npM"),i=s.n(l),o=s("lx39"),r=s.n(o);const n=e=>Boolean(e)&&(r()(e.value)||i()(e.value)),a=function(){let{options:e}=arguments.length>0&&void 0!==arguments[0]?arguments[0]:{};return Array.isArray(e)&&e.every(n)},u=e=>e.length===new Set(e).size,d=e=>e.flatMap(e=>n(e)?e:e.options),c=e=>(e=>e.every(n)||e.every(a))(e)&&(e=>u(d(e).map(e=>{let{value:t}=e;return t})))(e)&&(e=>u(e.filter(a).map(e=>{let{text:t}=e;return t})))(e)},"59DU":function(e,t){e.exports=function(e){return null==e}},crTv:function(e,t,s){"use strict";var l=s("3CjL"),i=s.n(l),o=s("o4PY"),r=s.n(o),n=s("59DU"),a=s.n(n),u=s("Qog8"),d=s("V5u/"),c=s("XBTk"),h=s("4lAS"),p=s("FkSe"),g=s("ehHk"),b=s("iN9h"),f=s("qaCH"),m=s("EldY"),v=s("zjrD"),y=s("3A1J"),x=s("s1D3"),S=s("Pyw5"),_=s.n(S);const I={name:"GlListboxSearchInput",components:{GlClearIconButton:y.a,GlIcon:x.a},model:{prop:"value",event:"input"},props:{value:{type:String,required:!1,default:""},placeholder:{type:String,required:!1,default:"Search"}},computed:{hasValue(){return Boolean(this.value.length)},inputListeners(){return{...this.$listeners,input:e=>{this.$emit("input",e.target.value)}}}},methods:{clearInput(){this.$emit("input",""),this.focusInput()},focusInput(){this.$refs.input.focus()}}};var B=_()({render:function(){var e=this,t=e.$createElement,s=e._self._c||t;return s("div",{staticClass:"gl-listbox-search"},[s("gl-icon",{staticClass:"gl-listbox-search-icon",attrs:{name:"search-sm",size:12}}),e._v(" "),s("input",e._g({ref:"input",staticClass:"gl-listbox-search-input",attrs:{type:"search","aria-label":e.placeholder,placeholder:e.placeholder},domProps:{value:e.value}},e.inputListeners)),e._v(" "),e.hasValue?s("gl-clear-icon-button",{staticClass:"gl-listbox-search-clear-button",on:{click:function(t){return t.stopPropagation(),e.clearInput.apply(null,arguments)}}}):e._e()],1)},staticRenderFns:[]},void 0,I,void 0,!1,void 0,!1,void 0,void 0,void 0);const w={name:"GlListboxGroup",props:{name:{type:String,required:!0},textSrOnly:{type:Boolean,required:!1,default:!1}},created(){this.nameId=r()("gl-listbox-group-")}};var C=_()({render:function(){var e=this,t=e.$createElement,s=e._self._c||t;return s("ul",{staticClass:"gl-mb-0 gl-pl-0",attrs:{role:"group","aria-labelledby":e.nameId}},[s("li",{staticClass:"gl-pb-2 gl-pl-4 gl-pt-3 gl-text-sm gl-font-bold gl-text-strong",class:{"gl-sr-only":e.textSrOnly},attrs:{id:e.nameId,role:"presentation"}},[e._t("group-label",(function(){return[e._v(e._s(e.name))]}))],2),e._v(" "),e._t("default")],2)},staticRenderFns:[]},void 0,w,void 0,!1,void 0,!1,void 0,void 0,void 0),O=s("3fen");const k=["gl-border-t","gl-border-t-dropdown","gl-pt-1","gl-mt-2"];const L={name:"GlCollapsibleListbox",HEADER_ITEMS_BORDER_CLASSES:["gl-border-b-1","gl-border-b-solid","gl-border-b-dropdown"],events:{GL_DROPDOWN_SHOWN:d.i,GL_DROPDOWN_HIDDEN:d.h},components:{GlBaseDropdown:f.b,GlListboxItem:v.a,GlListboxGroup:C,GlButton:h.a,GlSearchBoxByType:b.a,GlListboxSearchInput:B,GlLoadingIcon:p.a,GlIntersectionObserver:g.a},model:{prop:"selected",event:"select"},props:{items:{type:Array,required:!1,default:()=>[],validator:O.c},selected:{type:[Array,String,Number],required:!1,default:()=>[]},multiple:{type:Boolean,required:!1,default:!1},toggleText:{type:String,required:!1,default:""},textSrOnly:{type:Boolean,required:!1,default:!1},headerText:{type:String,required:!1,default:""},category:{type:String,required:!1,default:c.m.primary,validator:e=>e in c.m},variant:{type:String,required:!1,default:c.w.default,validator:e=>e in c.w},size:{type:String,required:!1,default:"medium",validator:e=>e in c.n},icon:{type:String,required:!1,default:""},disabled:{type:Boolean,required:!1,default:!1},loading:{type:Boolean,required:!1,default:!1},toggleClass:{type:[String,Array,Object],required:!1,default:null},noCaret:{type:Boolean,required:!1,default:!1},placement:{type:String,required:!1,default:"bottom-start",validator:e=>Object.keys(c.v).includes(e)},isCheckCentered:{type:Boolean,required:!1,default:!1},toggleAriaLabelledBy:{type:String,required:!1,default:null},listAriaLabelledBy:{type:String,required:!1,default:null},searchable:{type:Boolean,required:!1,default:!1},searching:{type:Boolean,required:!1,default:!1},infiniteScroll:{type:Boolean,required:!1,default:!1},totalItems:{type:Number,required:!1,default:null},infiniteScrollLoading:{type:Boolean,required:!1,default:!1},noResultsText:{type:String,required:!1,default:"No results found"},searchPlaceholder:{type:String,required:!1,default:"Search"},resetButtonLabel:{type:String,required:!1,default:""},showSelectAllButtonLabel:{type:String,required:!1,default:""},block:{type:Boolean,required:!1,default:!1},dropdownOffset:{type:[Number,Object],required:!1,default:void 0},fluidWidth:{type:Boolean,required:!1,default:!1},positioningStrategy:{type:String,required:!1,default:d.k,validator:e=>[d.k,d.l].includes(e)},startOpened:{type:Boolean,required:!1,default:!1},srOnlyResultsLabel:{type:Function,required:!1,default:Object(m.c)("GlCollapsibleListbox.srOnlyResultsLabel","%d result","%d results")}},data:()=>({selectedValues:[],toggleId:r()("dropdown-toggle-btn-"),listboxId:r()("listbox-"),nextFocusedItemIndex:null,searchStr:"",topBoundaryVisible:!0,bottomBoundaryVisible:!0}),computed:{listboxTag(){return!this.hasItems||Object(O.b)(this.items[0])?"ul":"div"},listboxClasses(){return{"top-scrim-visible":!this.topBoundaryVisible,"bottom-scrim-visible":!this.bottomBoundaryVisible,[d.f]:!0}},itemTag(){return"ul"===this.listboxTag?"li":"div"},flattenedOptions(){return Object(O.a)(this.items)},hasItems(){return this.items.length>0},listboxToggleText(){var e;return this.toggleText?this.toggleText:!this.multiple&&this.selectedValues.length?null===(e=this.flattenedOptions.find(e=>{let{value:t}=e;return t===this.selectedValues[0]}))||void 0===e?void 0:e.text:""},selectedIndices(){return this.selectedValues.map(e=>this.flattenedOptions.findIndex(t=>{let{value:s}=t;return s===e})).sort()},showList(){return this.flattenedOptions.length&&!this.searching},showNoResultsText(){return!this.flattenedOptions.length&&!this.searching},announceSRSearchResults(){return this.searchable&&!this.showNoResultsText},headerId(){return this.headerText&&r()("listbox-header-")},showResetButton(){return!!this.resetButtonLabel&&(!!this.hasItems&&(!(!this.selected||0===this.selected.length)&&!this.showSelectAllButton))},showSelectAllButton(){return!!this.showSelectAllButtonLabel&&(!!this.multiple&&(!!this.hasItems&&this.selected.length!==this.flattenedOptions.length))},showIntersectionObserver(){return this.infiniteScroll&&!this.infiniteScrollLoading&&!this.loading&&!this.searching},hasCustomToggle(){return Boolean(this.$scopedSlots.toggle)},hasSelection(){return Boolean(this.selectedValues.length)},toggleButtonClasses(){const e=[this.toggleClass];return this.hasSelection||e.push("!gl-text-subtle"),e},hasHeader(){return this.headerText||this.searchable},hasFooter(){return Boolean(this.$scopedSlots.footer)}},watch:{selected:{immediate:!0,handler(e){Array.isArray(e)?this.selectedValues=[...e]:this.selectedValues=a()(e)?[]:[e]}},items:{handler(){this.$nextTick(()=>{this.observeScroll()})}}},mounted(){this.startOpened&&this.open(),this.observeScroll()},beforeDestroy(){var e;null===(e=this.scrollObserver)||void 0===e||e.disconnect()},methods:{open(){this.$refs.baseDropdown.open()},close(){this.$refs.baseDropdown.close()},groupClasses:e=>0===e?null:k,onShow(){var e;this.searchable?this.focusSearchInput():this.focusItem(null!==(e=this.selectedIndices[0])&&void 0!==e?e:0,this.getFocusableListItemElements());this.$emit(d.i)},onHide(){this.$emit(d.h),this.nextFocusedItemIndex=null},onKeydown(e){const{code:t,target:s}=e,l=this.getFocusableListItemElements();if(l.length<1)return;let i=!0;const o=s.matches(".gl-listbox-search-input");if(t===d.j){if(o)return;this.focusItem(0,l)}else if(t===d.c){if(o)return;this.focusItem(l.length-1,l)}else if(t===d.b){if(o)return;this.searchable&&0===l.indexOf(s)?this.focusSearchInput():this.focusNextItem(e,l,-1)}else t===d.a?o?this.focusItem(0,l):this.focusNextItem(e,l,1):i=!1;i&&Object(u.k)(e)},getFocusableListItemElements(){var e;const t=null===(e=this.$refs.list)||void 0===e?void 0:e.querySelectorAll('[role="option"]');return Array.from(t||[])},focusNextItem(e,t,s){const{target:l}=e,o=t.indexOf(l),r=i()(o+s,0,t.length-1);this.focusItem(r,t)},focusItem(e,t){var s;this.nextFocusedItemIndex=e,null===(s=t[e])||void 0===s||s.focus()},focusSearchInput(){this.$refs.searchBox.focusInput()},onSelect(e,t){this.multiple?this.onMultiSelect(e.value,t):this.onSingleSelect(e.value,t)},isSelected(e){return this.selectedValues.some(t=>t===e.value)},isFocused(e){return this.nextFocusedItemIndex===this.flattenedOptions.indexOf(e)},onSingleSelect(e,t){t&&this.$emit("select",e),this.closeAndFocus()},onMultiSelect(e,t){t?this.$emit("select",[...this.selectedValues,e]):this.$emit("select",this.selectedValues.filter(t=>t!==e))},search(e){this.$emit("search",e)},onResetButtonClicked(){this.$emit("reset")},onSelectAllButtonClicked(){this.$emit("select-all")},closeAndFocus(){this.$refs.baseDropdown.closeAndFocus()},onIntersectionObserverAppear(){this.$emit("bottom-reached")},listboxItemMoreItemsAriaAttributes(e){return null===this.totalItems?{}:{"aria-setsize":this.totalItems,"aria-posinset":e+1}},observeScroll(){var e;const t={rootMargin:"8px",root:this.$refs.list,threshold:1};null===(e=this.scrollObserver)||void 0===e||e.disconnect();const s=new IntersectionObserver(e=>{e.forEach(e=>{var t;this[null===(t=e.target)||void 0===t?void 0:t.$__visibilityProp]=e.isIntersecting})},t),l=this.$refs["top-boundary"],i=this.$refs["bottom-boundary"];l&&(l.$__visibilityProp="topBoundaryVisible",s.observe(l)),i&&(i.$__visibilityProp="bottomBoundaryVisible",s.observe(i)),this.scrollObserver=s},isOption:O.b}};const $=_()({render:function(){var e=this,t=e.$createElement,s=e._self._c||t;return s("gl-base-dropdown",{ref:"baseDropdown",attrs:{"aria-haspopup":"listbox","aria-labelledby":e.toggleAriaLabelledBy,block:e.block,"toggle-id":e.toggleId,"toggle-text":e.listboxToggleText,"toggle-class":e.toggleButtonClasses,"text-sr-only":e.textSrOnly,category:e.category,variant:e.variant,size:e.size,icon:e.icon,disabled:e.disabled,loading:e.loading,"no-caret":e.noCaret,placement:e.placement,offset:e.dropdownOffset,"fluid-width":e.fluidWidth,"positioning-strategy":e.positioningStrategy},on:e._d({},[e.$options.events.GL_DROPDOWN_SHOWN,e.onShow,e.$options.events.GL_DROPDOWN_HIDDEN,e.onHide]),scopedSlots:e._u([e.hasCustomToggle?{key:"toggle",fn:function(){return[e._t("toggle")]},proxy:!0}:null],null,!0)},[e._v(" "),e.headerText?s("div",{staticClass:"gl-flex gl-min-h-8 gl-items-center !gl-p-4",class:e.$options.HEADER_ITEMS_BORDER_CLASSES},[s("div",{staticClass:"gl-grow gl-pr-2 gl-text-sm gl-font-bold gl-text-strong",attrs:{id:e.headerId,"data-testid":"listbox-header-text"}},[e._v("\n      "+e._s(e.headerText)+"\n    ")]),e._v(" "),e.showResetButton?s("gl-button",{staticClass:"!gl-m-0 !gl-w-auto gl-max-w-1/2 gl-flex-shrink-0 gl-text-ellipsis !gl-px-2 !gl-text-sm focus:!gl-shadow-inner-2-blue-400",attrs:{category:"tertiary",size:"small","data-testid":"listbox-reset-button"},on:{click:e.onResetButtonClicked}},[e._v("\n      "+e._s(e.resetButtonLabel)+"\n    ")]):e._e(),e._v(" "),e.showSelectAllButton?s("gl-button",{staticClass:"!gl-m-0 !gl-w-auto gl-max-w-1/2 gl-flex-shrink-0 gl-text-ellipsis !gl-px-2 !gl-text-sm focus:!gl-shadow-inner-2-blue-400",attrs:{category:"tertiary",size:"small","data-testid":"listbox-select-all-button"},on:{click:e.onSelectAllButtonClicked}},[e._v("\n      "+e._s(e.showSelectAllButtonLabel)+"\n    ")]):e._e()],1):e._e(),e._v(" "),e.searchable?s("div",{class:e.$options.HEADER_ITEMS_BORDER_CLASSES},[s("gl-listbox-search-input",{ref:"searchBox",class:{"gl-listbox-topmost":!e.headerText},attrs:{"data-testid":"listbox-search-input",placeholder:e.searchPlaceholder},on:{input:e.search,keydown:[function(t){if(!t.type.indexOf("key")&&e._k(t.keyCode,"enter",13,t.key,"Enter"))return null;t.preventDefault()},e.onKeydown]},model:{value:e.searchStr,callback:function(t){e.searchStr=t},expression:"searchStr"}}),e._v(" "),e.searching?s("gl-loading-icon",{staticClass:"gl-my-3",attrs:{"data-testid":"listbox-search-loader",size:"md"}}):e._e()],1):e._e(),e._v(" "),e.showList?s(e.listboxTag,{ref:"list",tag:"component",staticClass:"gl-new-dropdown-contents gl-new-dropdown-contents-with-scrim-overlay",class:e.listboxClasses,attrs:{id:e.listboxId,"aria-labelledby":e.listAriaLabelledBy||e.headerId||e.toggleId,role:"listbox",tabindex:"0"},on:{keydown:e.onKeydown}},[s(e.itemTag,{tag:"component",staticClass:"top-scrim-wrapper",attrs:{"aria-hidden":"true","data-testid":"top-scrim"}},[s("div",{staticClass:"top-scrim",class:{"top-scrim-light":!e.hasHeader,"top-scrim-dark":e.hasHeader}})]),e._v(" "),s(e.itemTag,{ref:"top-boundary",tag:"component",attrs:{"aria-hidden":"true"}}),e._v(" "),e._l(e.items,(function(t,l){return[e.isOption(t)?[s("gl-listbox-item",e._b({key:t.value,attrs:{"data-testid":"listbox-item-"+t.value,"is-selected":e.isSelected(t),"is-focused":e.isFocused(t),"is-check-centered":e.isCheckCentered},on:{select:function(s){return e.onSelect(t,s)}}},"gl-listbox-item",e.listboxItemMoreItemsAriaAttributes(l),!1),[e._t("list-item",(function(){return[e._v("\n            "+e._s(t.text)+"\n          ")]}),{item:t})],2)]:[s("gl-listbox-group",{key:t.text,class:e.groupClasses(l),attrs:{name:t.text,"text-sr-only":t.textSrOnly},scopedSlots:e._u([e.$scopedSlots["group-label"]?{key:"group-label",fn:function(){return[e._t("group-label",null,{group:t})]},proxy:!0}:null],null,!0)},[e._v(" "),e._l(t.options,(function(t){return s("gl-listbox-item",{key:t.value,attrs:{"data-testid":"listbox-item-"+t.value,"is-selected":e.isSelected(t),"is-focused":e.isFocused(t),"is-check-centered":e.isCheckCentered},on:{select:function(s){return e.onSelect(t,s)}}},[e._t("list-item",(function(){return[e._v("\n              "+e._s(t.text)+"\n            ")]}),{item:t})],2)}))],2)]]})),e._v(" "),e.infiniteScrollLoading?s(e.itemTag,{tag:"component"},[s("gl-loading-icon",{staticClass:"gl-my-3",attrs:{"data-testid":"listbox-infinite-scroll-loader",size:"md"}})],1):e._e(),e._v(" "),e.showIntersectionObserver?s("gl-intersection-observer",{on:{appear:e.onIntersectionObserverAppear}}):e._e(),e._v(" "),s(e.itemTag,{ref:"bottom-boundary",tag:"component",attrs:{"aria-hidden":"true"}}),e._v(" "),s(e.itemTag,{tag:"component",staticClass:"bottom-scrim-wrapper",attrs:{"aria-hidden":"true","data-testid":"bottom-scrim"}},[s("div",{staticClass:"bottom-scrim",class:{"!gl-rounded-none":e.hasFooter}})])],2):e._e(),e._v(" "),e.announceSRSearchResults?s("span",{staticClass:"gl-sr-only",attrs:{"data-testid":"listbox-number-of-results","aria-live":"assertive"}},[e._t("search-summary-sr-only",(function(){return[e._v("\n      "+e._s(e.srOnlyResultsLabel(e.flattenedOptions.length))+"\n    ")]}))],2):e.showNoResultsText?s("div",{staticClass:"gl-py-3 gl-pl-7 gl-pr-5 gl-text-base gl-text-subtle",attrs:{"aria-live":"assertive","data-testid":"listbox-no-results-text"}},[e._v("\n    "+e._s(e.noResultsText)+"\n  ")]):e._e(),e._v(" "),e._t("footer")],2)},staticRenderFns:[]},void 0,L,void 0,!1,void 0,!1,void 0,void 0,void 0);t.a=$},iN9h:function(e,t,s){"use strict";var l=s("3A1J"),i=s("PrLL"),o=s("s1D3"),r=s("FkSe"),n=s("EldY"),a=s("Pyw5"),u=s.n(a);const d={name:"GlSearchboxByType",components:{GlClearIconButton:l.a,GlIcon:o.a,GlFormInput:i.a,GlLoadingIcon:r.a},inheritAttrs:!1,model:{prop:"value",event:"input"},props:{value:{type:String,required:!1,default:""},borderless:{type:Boolean,required:!1,default:!1},clearButtonTitle:{type:String,required:!1,default:()=>Object(n.b)("GlSearchBoxByType.clearButtonTitle","Clear")},disabled:{type:Boolean,required:!1,default:!1},isLoading:{type:Boolean,required:!1,default:!1},tooltipContainer:{required:!1,default:!1,validator:e=>!1===e||"string"==typeof e||e instanceof HTMLElement}},computed:{inputAttributes(){const e={type:"search",placeholder:Object(n.b)("GlSearchBoxByType.input.placeholder","Search"),...this.$attrs};return e["aria-label"]||(e["aria-label"]=e.placeholder),e},hasValue(){return Boolean(this.value.length)},inputListeners(){return{...this.$listeners,input:this.onInput,focusin:this.onFocusin,focusout:this.onFocusout}},showClearButton(){return this.hasValue&&!this.disabled}},methods:{isInputOrClearButton(e){var t,s;return e===(null===(t=this.$refs.input)||void 0===t?void 0:t.$el)||e===(null===(s=this.$refs.clearButton)||void 0===s?void 0:s.$el)},clearInput(){this.onInput(""),this.focusInput()},focusInput(){this.$refs.input.$el.focus()},onInput(e){this.$emit("input",e)},onFocusout(e){const{relatedTarget:t}=e;this.isInputOrClearButton(t)||this.$emit("focusout",e)},onFocusin(e){const{relatedTarget:t}=e;this.isInputOrClearButton(t)||this.$emit("focusin",e)}}};const c=u()({render:function(){var e=this,t=e.$createElement,s=e._self._c||t;return s("div",{staticClass:"gl-search-box-by-type"},[s("gl-icon",{staticClass:"gl-search-box-by-type-search-icon",attrs:{name:"search",variant:"subtle"}}),e._v(" "),s("gl-form-input",e._g(e._b({ref:"input",class:{"gl-search-box-by-type-input":!e.borderless,"gl-search-box-by-type-input-borderless":e.borderless},attrs:{value:e.value,disabled:e.disabled}},"gl-form-input",e.inputAttributes,!1),e.inputListeners)),e._v(" "),e.isLoading||e.showClearButton?s("div",{staticClass:"gl-search-box-by-type-right-icons"},[e.isLoading?s("gl-loading-icon",{staticClass:"gl-search-box-by-type-loading-icon"}):e._e(),e._v(" "),e.showClearButton?s("gl-clear-icon-button",{ref:"clearButton",staticClass:"gl-search-box-by-type-clear gl-clear-icon-button",attrs:{title:e.clearButtonTitle,"tooltip-container":e.tooltipContainer},on:{click:function(t){return t.stopPropagation(),e.clearInput.apply(null,arguments)},focusin:e.onFocusin,focusout:e.onFocusout}}):e._e()],1):e._e()],1)},staticRenderFns:[]},void 0,d,void 0,!1,void 0,!1,void 0,void 0,void 0);t.a=c},zjrD:function(e,t,s){"use strict";var l=s("s1D3"),i=s("V5u/"),o=s("Qog8"),r=s("Pyw5"),n=s.n(r);const a={name:"GlListboxItem",components:{GlIcon:l.a},props:{isSelected:{type:Boolean,default:!1,required:!1},isFocused:{type:Boolean,default:!1,required:!1},isCheckCentered:{type:Boolean,required:!1,default:!1}},computed:{checkedClasses(){return this.isCheckCentered?"":"gl-mt-3 gl-self-start"}},methods:{toggleSelection(){this.$emit("select",!this.isSelected)},onKeydown(e){const{code:t}=e;t!==i.d&&t!==i.m||(Object(o.k)(e),this.toggleSelection())}}};const u=n()({render:function(){var e=this,t=e.$createElement,s=e._self._c||t;return s("li",{staticClass:"gl-new-dropdown-item",attrs:{role:"option",tabindex:e.isFocused?0:-1,"aria-selected":e.isSelected},on:{click:e.toggleSelection,keydown:e.onKeydown}},[s("span",{staticClass:"gl-new-dropdown-item-content"},[s("gl-icon",{class:["gl-new-dropdown-item-check-icon",{"gl-invisible":!e.isSelected},e.checkedClasses],attrs:{name:"mobile-issue-close","data-testid":"dropdown-item-checkbox"}}),e._v(" "),s("span",{staticClass:"gl-new-dropdown-item-text-wrapper"},[e._t("default")],2)],1)])},staticRenderFns:[]},void 0,a,void 0,!1,void 0,!1,void 0,void 0,void 0);t.a=u}}]);
//# sourceMappingURL=26.382c9872.chunk.js.map