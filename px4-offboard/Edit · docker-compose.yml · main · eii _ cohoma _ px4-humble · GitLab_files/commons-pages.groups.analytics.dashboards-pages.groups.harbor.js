(this.webpackJsonp=this.webpackJsonp||[]).push([[21],{"7z1+":function(e,t,r){"use strict";r.d(t,"a",(function(){return h})),r.d(t,"b",(function(){return u})),r.d(t,"c",(function(){return d})),r.d(t,"d",(function(){return c}));var i=r("ewH8"),s=r("KFC0"),o=r.n(s),n=r("lx39"),l=r.n(n),a=r("BglX");const d=e=>Boolean(e)&&(e=>{var t;return(null==e||null===(t=e.text)||void 0===t?void 0:t.length)>0&&!Array.isArray(null==e?void 0:e.items)})(e),u=e=>Boolean(e)&&Array.isArray(e.items)&&Boolean(e.items.length)&&e.items.every(d),c=e=>e.every(d)||e.every(u),m=e=>{const t=e();if(!Array.isArray(t))return!1;const r=t.filter(e=>e.tag);return r.length&&r.every(e=>(e=>{var t,r;return Boolean(e)&&(r=(null===(t=e.componentOptions)||void 0===t?void 0:t.tag)||e.tag,["gl-disclosure-dropdown-group","gl-disclosure-dropdown-item","li"].includes(r))})(e))},p=e=>{const t=e(),r=t.find(e=>Array.isArray(e.children)&&e.children.length);return(r?r.children:t).filter(e=>!l()(e.text)||e.text.trim().length>0).every(e=>(e=>{var t;return[a.c,a.b].includes(null===(t=e.type)||void 0===t?void 0:t.name)||"li"===e.type})(e))},h=e=>!!o()(e)&&(i.default.version.startsWith("3")?p(e):m(e))},BglX:function(e,t,r){"use strict";r.d(t,"a",(function(){return o})),r.d(t,"b",(function(){return s})),r.d(t,"c",(function(){return i}));const i="GlDisclosureDropdownItem",s="GlDisclosureDropdownGroup",o={top:"top",bottom:"bottom"}},Bo17:function(e,t,r){"use strict";var i=r("3CjL"),s=r.n(i),o=r("o4PY"),n=r.n(o),l=r("Qog8"),a=r("V5u/"),d=r("XBTk"),u=r("qaCH"),c=r("XiQx"),m=r("fSQg"),p=r("7z1+"),h=r("Pyw5"),b=r.n(h);const g="."+u.a,f="."+c.a;const v={name:"GlDisclosureDropdown",events:{GL_DROPDOWN_SHOWN:a.i,GL_DROPDOWN_HIDDEN:a.h,GL_DROPDOWN_BEFORE_CLOSE:a.e,GL_DROPDOWN_FOCUS_CONTENT:a.g},components:{GlBaseDropdown:u.b,GlDisclosureDropdownItem:c.b,GlDisclosureDropdownGroup:m.a},props:{items:{type:Array,required:!1,default:()=>[],validator:p.d},toggleText:{type:String,required:!1,default:""},textSrOnly:{type:Boolean,required:!1,default:!1},category:{type:String,required:!1,default:d.m.primary,validator:e=>e in d.m},variant:{type:String,required:!1,default:d.w.default,validator:e=>e in d.w},size:{type:String,required:!1,default:"medium",validator:e=>e in d.n},icon:{type:String,required:!1,default:""},disabled:{type:Boolean,required:!1,default:!1},loading:{type:Boolean,required:!1,default:!1},toggleId:{type:String,required:!1,default:()=>n()("dropdown-toggle-btn-")},toggleClass:{type:[String,Array,Object],required:!1,default:null},noCaret:{type:Boolean,required:!1,default:!1},placement:{type:String,required:!1,default:"bottom-start",validator:e=>Object.keys(d.v).includes(e)},toggleAriaLabelledBy:{type:String,required:!1,default:null},listAriaLabelledBy:{type:String,required:!1,default:null},block:{type:Boolean,required:!1,default:!1},dropdownOffset:{type:[Number,Object],required:!1,default:void 0},fluidWidth:{type:Boolean,required:!1,default:!1},autoClose:{type:Boolean,required:!1,default:!0},positioningStrategy:{type:String,required:!1,default:a.k,validator:e=>[a.k,a.l].includes(e)},startOpened:{type:Boolean,required:!1,default:!1}},data:()=>({disclosureId:n()("disclosure-"),nextFocusedItemIndex:null}),computed:{disclosureTag(){var e;return null!==(e=this.items)&&void 0!==e&&e.length||Object(p.a)(this.$scopedSlots.default||this.$slots.default)?"ul":"div"},hasCustomToggle(){return Boolean(this.$scopedSlots.toggle)}},mounted(){this.startOpened&&this.open()},methods:{open(){this.$refs.baseDropdown.open()},close(){this.$refs.baseDropdown.close()},onShow(){this.$emit(a.i)},onBeforeClose(e){this.$emit(a.e,e)},onHide(){this.$emit(a.h),this.nextFocusedItemIndex=null},onKeydown(e){const{code:t}=e,r=this.getFocusableListItemElements();if(r.length<1)return;let i=!0;t===a.j?this.focusItem(0,r):t===a.c?this.focusItem(r.length-1,r):t===a.b?this.focusNextItem(e,r,-1):t===a.a?this.focusNextItem(e,r,1):t===a.d||t===a.m?this.handleAutoClose(e):i=!1,i&&Object(l.k)(e)},getFocusableListItemElements(){var e;const t=null===(e=this.$refs.content)||void 0===e?void 0:e.querySelectorAll(f);return Object(l.c)(Array.from(t||[]))},focusNextItem(e,t,r){const{target:i}=e,o=t.indexOf(i),n=s()(o+r,0,t.length-1);this.focusItem(n,t)},focusItem(e,t){var r;this.nextFocusedItemIndex=e,null===(r=t[e])||void 0===r||r.focus()},closeAndFocus(){this.$refs.baseDropdown.closeAndFocus()},handleAction(e){window.requestAnimationFrame(()=>{this.$emit("action",e)})},handleAutoClose(e){this.autoClose&&e.target.closest(f)&&e.target.closest(g)===this.$refs.baseDropdown.$el&&this.closeAndFocus()},uniqueItemId:()=>n()("disclosure-item-"),isItem:p.c},GL_DROPDOWN_CONTENTS_CLASS:a.f};const y=b()({render:function(){var e=this,t=e.$createElement,r=e._self._c||t;return r("gl-base-dropdown",{ref:"baseDropdown",staticClass:"gl-disclosure-dropdown",attrs:{"aria-labelledby":e.toggleAriaLabelledBy,"toggle-id":e.toggleId,"toggle-text":e.toggleText,"toggle-class":e.toggleClass,"text-sr-only":e.textSrOnly,category:e.category,variant:e.variant,size:e.size,icon:e.icon,disabled:e.disabled,loading:e.loading,"no-caret":e.noCaret,placement:e.placement,block:e.block,offset:e.dropdownOffset,"fluid-width":e.fluidWidth,"positioning-strategy":e.positioningStrategy},on:e._d({},[e.$options.events.GL_DROPDOWN_SHOWN,e.onShow,e.$options.events.GL_DROPDOWN_HIDDEN,e.onHide,e.$options.events.GL_DROPDOWN_BEFORE_CLOSE,e.onBeforeClose,e.$options.events.GL_DROPDOWN_FOCUS_CONTENT,e.onKeydown]),scopedSlots:e._u([e.hasCustomToggle?{key:"toggle",fn:function(){return[e._t("toggle")]},proxy:!0}:null],null,!0)},[e._v(" "),e._t("header"),e._v(" "),r(e.disclosureTag,{ref:"content",tag:"component",class:e.$options.GL_DROPDOWN_CONTENTS_CLASS,attrs:{id:e.disclosureId,"aria-labelledby":e.listAriaLabelledBy||e.toggleId,"data-testid":"disclosure-content",tabindex:"-1"},on:{keydown:e.onKeydown,click:e.handleAutoClose}},[e._t("default",(function(){return[e._l(e.items,(function(t,i){return[e.isItem(t)?[r("gl-disclosure-dropdown-item",{key:e.uniqueItemId(),attrs:{item:t},on:{action:e.handleAction},scopedSlots:e._u(["list-item"in e.$scopedSlots?{key:"list-item",fn:function(){return[e._t("list-item",null,{item:t})]},proxy:!0}:null],null,!0)})]:[r("gl-disclosure-dropdown-group",{key:t.name,attrs:{bordered:0!==i,group:t},on:{action:e.handleAction},scopedSlots:e._u([e.$scopedSlots["group-label"]?{key:"group-label",fn:function(){return[e._t("group-label",null,{group:t})]},proxy:!0}:null],null,!0)},[e._v(" "),e.$scopedSlots["list-item"]?e._l(t.items,(function(t){return r("gl-disclosure-dropdown-item",{key:e.uniqueItemId(),attrs:{item:t},on:{action:e.handleAction},scopedSlots:e._u([{key:"list-item",fn:function(){return[e._t("list-item",null,{item:t})]},proxy:!0}],null,!0)})})):e._e()],2)]]}))]}))],2),e._v(" "),e._t("footer")],2)},staticRenderFns:[]},void 0,v,void 0,!1,void 0,!1,void 0,void 0,void 0);t.a=y},XiQx:function(e,t,r){"use strict";r.d(t,"a",(function(){return u}));var i=r("0M2I"),s=r("V5u/"),o=r("Qog8"),n=r("7z1+"),l=r("BglX"),a=r("Pyw5"),d=r.n(a);const u="gl-new-dropdown-item";const c={name:l.c,ITEM_CLASS:u,components:{BLink:i.a},props:{item:{type:Object,required:!1,default:null,validator:n.c}},computed:{isLink(){var e,t;return"string"==typeof(null===(e=this.item)||void 0===e?void 0:e.href)||"string"==typeof(null===(t=this.item)||void 0===t?void 0:t.to)},isCustomContent(){return Boolean(this.$scopedSlots.default)},itemComponent(){const{item:e}=this;return this.isLink?{is:i.a,attrs:{href:e.href,to:e.to,...e.extraAttrs},listeners:{click:this.action}}:{is:"button",attrs:{...null==e?void 0:e.extraAttrs,type:"button"},listeners:{click:()=>{var t;null==e||null===(t=e.action)||void 0===t||t.call(void 0,e),this.action()}}}},listIndex(){var e,t;return null!==(e=this.item)&&void 0!==e&&null!==(t=e.extraAttrs)&&void 0!==t&&t.disabled?null:0},componentIndex(){var e,t;return null!==(e=this.item)&&void 0!==e&&null!==(t=e.extraAttrs)&&void 0!==t&&t.disabled?null:-1},wrapperClass(){var e,t;return null!==(e=null===(t=this.item)||void 0===t?void 0:t.wrapperClass)&&void 0!==e?e:""},wrapperListeners(){const e={keydown:this.onKeydown};return this.isCustomContent&&(e.click=this.action),e}},methods:{onKeydown(e){const{code:t}=e;if(t===s.d||t===s.m)if(this.isCustomContent)this.action();else{Object(o.k)(e);const t=new MouseEvent("click",{bubbles:!0,cancelable:!0});var r;if(this.isLink)this.$refs.item.$el.dispatchEvent(t);else null===(r=this.$refs.item)||void 0===r||r.dispatchEvent(t)}},action(){this.$emit("action",this.item)}}};const m=d()({render:function(){var e=this,t=e.$createElement,r=e._self._c||t;return r("li",e._g({class:[e.$options.ITEM_CLASS,e.wrapperClass],attrs:{tabindex:e.listIndex,"data-testid":"disclosure-dropdown-item"}},e.wrapperListeners),[e._t("default",(function(){return[r(e.itemComponent.is,e._g(e._b({ref:"item",tag:"component",staticClass:"gl-new-dropdown-item-content",attrs:{tabindex:e.componentIndex}},"component",e.itemComponent.attrs,!1),e.itemComponent.listeners),[r("span",{staticClass:"gl-new-dropdown-item-text-wrapper"},[e._t("list-item",(function(){return[e._v("\n          "+e._s(e.item.text)+"\n        ")]}))],2)])]}))],2)},staticRenderFns:[]},void 0,c,void 0,!1,void 0,!1,void 0,void 0,void 0);t.b=m},fSQg:function(e,t,r){"use strict";var i=r("o4PY"),s=r.n(i),o=r("XiQx"),n=r("7z1+"),l=r("BglX"),a=r("Pyw5"),d=r.n(a);const u={[l.a.top]:"gl-border-t gl-border-t-dropdown gl-pt-2 gl-mt-2",[l.a.bottom]:"gl-border-b gl-border-b-dropdown gl-pb-2 gl-mb-2"};const c={name:l.b,components:{GlDisclosureDropdownItem:o.b},props:{group:{type:Object,required:!1,default:null,validator:n.b},bordered:{type:Boolean,required:!1,default:!1},borderPosition:{type:String,required:!1,default:l.a.top,validator:e=>Object.keys(l.a).includes(e)}},computed:{borderClass(){return this.bordered?u[this.borderPosition]:null},showHeader(){var e;return this.$scopedSlots["group-label"]||(null===(e=this.group)||void 0===e?void 0:e.name)},groupLabeledBy(){return this.showHeader?this.nameId:null}},created(){this.nameId=s()("gl-disclosure-dropdown-group-")},methods:{handleAction(e){this.$emit("action",e)},uniqueItemId:()=>s()("disclosure-item-")}};const m=d()({render:function(){var e=this,t=e.$createElement,r=e._self._c||t;return r("li",{class:e.borderClass},[e.showHeader?r("div",{staticClass:"gl-py-2 gl-pl-4 gl-text-sm gl-font-bold gl-text-strong",attrs:{id:e.nameId,"aria-hidden":"true"}},[e._t("group-label",(function(){return[e._v(e._s(e.group.name))]}))],2):e._e(),e._v(" "),r("ul",{staticClass:"gl-mb-0 gl-list-none gl-pl-0",attrs:{"aria-labelledby":e.groupLabeledBy}},[e._t("default",(function(){return e._l(e.group.items,(function(t){return r("gl-disclosure-dropdown-item",{key:e.uniqueItemId(),attrs:{item:t},on:{action:e.handleAction},scopedSlots:e._u([{key:"list-item",fn:function(){return[e._t("list-item",null,{item:t})]},proxy:!0}],null,!0)})}))}))],2)])},staticRenderFns:[]},void 0,c,void 0,!1,void 0,!1,void 0,void 0,void 0);t.a=m},lbc5:function(e,t,r){"use strict";var i=r("Tmea"),s=r.n(i),o=r("9k56"),n=r("lgrP"),l=r("3hkr"),a=r("jIK5"),d=r("gZSI"),u=r("hII5"),c=r("TnX6"),m=r("Xhk9"),p=r("VuSA"),h=r("0M2I");const b=Object(u.c)(Object(p.m)({...Object(p.j)(h.b,["event","routerTag"]),ariaCurrent:Object(u.b)(a.r,"location"),html:Object(u.b)(a.r),text:Object(u.b)(a.r)}),l.d),g=Object(o.c)({name:l.d,functional:!0,props:b,render(e,t){let{props:r,data:i,children:s}=t;const{active:o}=r,l=o?"span":h.a,a={attrs:{"aria-current":o?r.ariaCurrent:null},props:Object(u.d)(b,r)};return s||(a.domProps=Object(m.a)(r.html,r.text)),e(l,Object(n.a)(i,a),s)}}),f=Object(u.c)(b,l.c),v=Object(o.c)({name:l.c,functional:!0,props:f,render(e,t){let{props:r,data:i,children:s}=t;return e("li",Object(n.a)(i,{staticClass:"breadcrumb-item",class:{active:r.active}}),[e(g,{props:r},s)])}}),y=Object(u.c)({items:Object(u.b)(a.b)},l.b),w=Object(o.c)({name:l.b,functional:!0,props:y,render(e,t){let{props:r,data:i,children:s}=t;const{items:o}=r;let l=s;if(Object(d.a)(o)){let t=!1;l=o.map((r,i)=>{Object(d.i)(r)||(r={text:Object(c.e)(r)});let{active:s}=r;return s&&(t=!0),s||t||(s=i+1===o.length),e(v,{props:{...r,active:s}})})}return e("ol",Object(n.a)(i,{staticClass:"breadcrumb"}),l)}});var _=r("EldY"),O=r("tbP8"),I=r("Bo17"),C=r("z1xw"),x=r("Pyw5"),S=r.n(x);const B={name:"GlBreadcrumbItem",components:{BLink:h.a},inheritAttrs:!1,props:{text:{type:String,required:!1,default:null},to:{type:[String,Object],required:!1,default:null},href:{type:String,required:!1,default:null},ariaCurrent:{type:[String,Boolean],required:!1,default:!1,validator:e=>-1!==[!1,"page"].indexOf(e)}}};const A={name:"GlBreadcrumb",components:{BBreadcrumb:w,GlBreadcrumbItem:S()({render:function(){var e=this,t=e.$createElement,r=e._self._c||t;return r("li",{staticClass:"gl-breadcrumb-item"},[r("b-link",{attrs:{href:e.href,to:e.to,"aria-current":e.ariaCurrent}},[e._t("default",(function(){return[e._v(e._s(e.text))]}))],2)],1)},staticRenderFns:[]},void 0,B,void 0,!1,void 0,!1,void 0,void 0,void 0),GlAvatar:O.a,GlDisclosureDropdown:I.a},directives:{GlTooltip:C.a},inheritAttrs:!1,props:{items:{type:Array,required:!0,default:()=>[{text:"",href:""}],validator:e=>e.every(e=>{const t=Object.keys(e);return t.includes("text")&&(t.includes("href")||t.includes("to"))})},ariaLabel:{type:String,required:!1,default:"Breadcrumb"},showMoreLabel:{type:String,required:!1,default:()=>Object(_.b)("GlBreadcrumb.showMoreLabel","Show more breadcrumbs")},autoResize:{type:Boolean,required:!1,default:!0}},data(){return{fittingItems:[...this.items],overflowingItems:[],totalBreadcrumbsWidth:0,widthPerItem:[],resizeDone:!1}},computed:{hasCollapsible(){return this.overflowingItems.length>0},breadcrumbStyle(){return this.resizeDone?{}:{opacity:0}},itemStyle(){return this.resizeDone&&1===this.fittingItems.length?{"flex-shrink":1,"text-overflow":"ellipsis","overflow-x":"hidden","text-wrap":"nowrap"}:{}}},watch:{items:{handler:"measureAndMakeBreadcrumbsFit",deep:!0},autoResize(e){e?this.enableAutoResize():this.disableAutoResize()}},created(){this.debounceMakeBreadcrumbsFit=s()(this.makeBreadcrumbsFit,25)},mounted(){this.autoResize?this.enableAutoResize():this.resizeDone=!0},beforeDestroy(){this.disableAutoResize()},methods:{resetItems(){this.fittingItems=[...this.items],this.overflowingItems=[]},async measureAndMakeBreadcrumbsFit(){this.resetItems(),this.autoResize&&(this.resizeDone=!1,await this.$nextTick(),this.totalBreadcrumbsWidth=0,this.$refs.breadcrumbs&&(this.$refs.breadcrumbs.forEach((e,t)=>{const r=e.$el.clientWidth;this.totalBreadcrumbsWidth+=r,this.widthPerItem[t]=r}),this.makeBreadcrumbsFit()))},makeBreadcrumbsFit(){this.resizeDone=!1,this.resetItems();const e=this.$el.clientWidth;if(this.totalBreadcrumbsWidth>e){const t=0,r=this.items.length-1;let i=this.totalBreadcrumbsWidth;for(let s=t;s<r&&(this.overflowingItems.push(this.fittingItems[t]),this.fittingItems.splice(t,1),i-=this.widthPerItem[s],!(i+40<e));s+=1);}this.resizeDone=!0},isLastItem(e){return e===this.fittingItems.length-1},getAriaCurrentAttr(e){return!!this.isLastItem(e)&&"page"},enableAutoResize(){this.resizeObserver||(this.resizeObserver=new ResizeObserver(this.debounceMakeBreadcrumbsFit)),this.resizeObserver.observe(this.$el),this.measureAndMakeBreadcrumbsFit()},disableAutoResize(){this.resizeObserver&&(this.resizeObserver.unobserve(this.$el),this.resizeObserver=null),this.resetItems()}}};const D=S()({render:function(){var e=this,t=e.$createElement,r=e._self._c||t;return r("nav",{staticClass:"gl-breadcrumbs",style:e.breadcrumbStyle,attrs:{"aria-label":e.ariaLabel}},[r("b-breadcrumb",e._g(e._b({staticClass:"gl-breadcrumb-list"},"b-breadcrumb",e.$attrs,!1),e.$listeners),[e.hasCollapsible?r("li",{staticClass:"gl-breadcrumb-item"},[r("gl-disclosure-dropdown",{attrs:{items:e.overflowingItems,"toggle-text":e.showMoreLabel,"fluid-width":"","text-sr-only":"","no-caret":"",icon:"ellipsis_h",size:"small"}})],1):e._e(),e._v(" "),e._l(e.fittingItems,(function(t,i){return r("gl-breadcrumb-item",{ref:"breadcrumbs",refInFor:!0,style:e.itemStyle,attrs:{text:t.text,href:t.href,to:t.to,"aria-current":e.getAriaCurrentAttr(i)}},[t.avatarPath?r("gl-avatar",{staticClass:"gl-breadcrumb-avatar-tile gl-border gl-mr-2 !gl-rounded-base",attrs:{src:t.avatarPath,size:16,"aria-hidden":"true",shape:"rect","data-testid":"avatar"}}):e._e(),r("span",[e._v(e._s(t.text))])],1)}))],2)],1)},staticRenderFns:[]},void 0,A,void 0,!1,void 0,!1,void 0,void 0,void 0);t.a=D}}]);
//# sourceMappingURL=commons-pages.groups.analytics.dashboards-pages.groups.harbor.repositories-pages.groups.iteration_ca-43c0ecf9.cd411fdc.chunk.js.map