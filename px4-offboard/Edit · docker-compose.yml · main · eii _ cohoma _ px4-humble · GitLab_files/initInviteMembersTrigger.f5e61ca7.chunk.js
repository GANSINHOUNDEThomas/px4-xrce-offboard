(this.webpackJsonp=this.webpackJsonp||[]).push([[587,1222],{"6fAW":function(e,t,n){"use strict";n.r(t),n.d(t,"default",(function(){return o}));n("ZzK0"),n("z6RN"),n("BzOf");var r=n("ewH8"),i=n("LgEk");function o(){const e=document.querySelectorAll(".js-invite-members-trigger");return!!e&&e.forEach((function(e){return new r.default({el:e,name:"InviteMembersTriggerRoot",render:function(t){return t(i.default,{props:{...e.dataset}})}})}))}},"7xOh":function(e,t,n){"use strict";n("Tznw"),n("IYH6"),n("6yen"),n("OeRx"),n("l/dT"),n("RqS2"),n("Zy7a"),n("cjZU"),n("OAhk"),n("X42P"),n("mHhP"),n("fn0I"),n("UB/6"),n("imhG"),n("v2fZ"),n("UezY"),n("z6RN"),n("hG7+"),n("ZzK0"),n("BzOf");class r{constructor(){this.$_all=new Map}dispose(){this.$_all.clear()}$on(e,t){const n=this.$_all.get(e);n&&n.push(t)||this.$_all.set(e,[t])}$off(e,t){const n=this.$_all.get(e)||[],r=t?n.filter((function(e){return e!==t})):[];r.length?this.$_all.set(e,r):this.$_all.delete(e)}$once(e,t){var n=this;const r=function(...i){n.$off(e,r),t(...i)};this.$on(e,r)}$emit(e,...t){(this.$_all.get(e)||[]).forEach((function(e){e(...t)}))}}t.a=function(){return new r}},"7z1+":function(e,t,n){"use strict";n.d(t,"a",(function(){return g})),n.d(t,"b",(function(){return c})),n.d(t,"c",(function(){return l})),n.d(t,"d",(function(){return d}));var r=n("ewH8"),i=n("KFC0"),o=n.n(i),s=n("lx39"),u=n.n(s),a=n("BglX");const l=e=>Boolean(e)&&(e=>{var t;return(null==e||null===(t=e.text)||void 0===t?void 0:t.length)>0&&!Array.isArray(null==e?void 0:e.items)})(e),c=e=>Boolean(e)&&Array.isArray(e.items)&&Boolean(e.items.length)&&e.items.every(l),d=e=>e.every(l)||e.every(c),m=e=>{const t=e();if(!Array.isArray(t))return!1;const n=t.filter(e=>e.tag);return n.length&&n.every(e=>(e=>{var t,n;return Boolean(e)&&(n=(null===(t=e.componentOptions)||void 0===t?void 0:t.tag)||e.tag,["gl-disclosure-dropdown-group","gl-disclosure-dropdown-item","li"].includes(n))})(e))},p=e=>{const t=e(),n=t.find(e=>Array.isArray(e.children)&&e.children.length);return(n?n.children:t).filter(e=>!u()(e.text)||e.text.trim().length>0).every(e=>(e=>{var t;return[a.c,a.b].includes(null===(t=e.type)||void 0===t?void 0:t.name)||"li"===e.type})(e))},g=e=>!!o()(e)&&(r.default.version.startsWith("3")?p(e):m(e))},A2UA:function(e,t,n){"use strict";var r=n("7xOh");t.a=Object(r.a)()},BglX:function(e,t,n){"use strict";n.d(t,"a",(function(){return o})),n.d(t,"b",(function(){return i})),n.d(t,"c",(function(){return r}));const r="GlDisclosureDropdownItem",i="GlDisclosureDropdownGroup",o={top:"top",bottom:"bottom"}},LgEk:function(e,t,n){"use strict";n.r(t);var r=n("4lAS"),i=n("MV2A"),o=n("wP8z"),s=n("XiQx"),u=n("/lV4"),a=n("A2UA"),l=n("i7S8"),c={components:{GlButton:r.a,GlLink:i.a,GlDropdownItem:o.a,GlDisclosureDropdownItem:s.b},props:{displayText:{type:String,required:!1,default:Object(u.s__)("InviteMembers|Invite team members")},icon:{type:String,required:!1,default:""},classes:{type:String,required:!1,default:""},variant:{type:String,required:!1,default:void 0},category:{type:String,required:!1,default:void 0},triggerSource:{type:String,required:!0},triggerElement:{type:String,required:!1,default:"button"}},computed:{componentAttributes(){return{class:this.classes,"data-testid":"invite-members-button"}},item(){return{text:this.displayText}},isButtonTrigger(){return this.triggerElement===l.F},isWithEmojiTrigger(){return this.triggerElement===l.I},isDropdownWithEmojiTrigger(){return this.triggerElement===l.H},isDisclosureTrigger(){return this.triggerElement===l.G}},methods:{openModal(){a.a.$emit("openModal",{source:this.triggerSource})},handleDisclosureDropdownAction(){this.openModal(),this.$emit("modal-opened")}}},d=n("tBpV"),m=Object(d.a)(c,(function(){var e=this,t=e._self._c;return e.isButtonTrigger?t("gl-button",e._b({attrs:{variant:e.variant,category:e.category,icon:e.icon},on:{click:e.openModal}},"gl-button",e.componentAttributes,!1),[e._v("\n  "+e._s(e.displayText)+"\n")]):e.isWithEmojiTrigger?t("gl-link",e._b({on:{click:e.openModal}},"gl-link",e.componentAttributes,!1),[e._v("\n  "+e._s(e.displayText)+"\n  "),t("gl-emoji",{staticClass:"gl-mr-1 gl-align-baseline gl-text-size-reset",attrs:{"data-name":e.icon}})],1):e.isDropdownWithEmojiTrigger?t("gl-dropdown-item",e._b({on:{click:e.openModal}},"gl-dropdown-item",e.componentAttributes,!1),[e._v("\n  "+e._s(e.displayText)+"\n  "),t("gl-emoji",{staticClass:"gl-mr-1 gl-align-baseline gl-text-size-reset",attrs:{"data-name":e.icon}})],1):e.isDisclosureTrigger?t("gl-disclosure-dropdown-item",e._b({attrs:{item:e.item},on:{action:e.handleDisclosureDropdownAction}},"gl-disclosure-dropdown-item",e.componentAttributes,!1)):t("gl-link",e._b({attrs:{"data-is-link":"true"},on:{click:e.openModal}},"gl-link",e.componentAttributes,!1),[e._v("\n  "+e._s(e.displayText)+"\n")])}),[],!1,null,null,null);t.default=m.exports},"V5u/":function(e,t,n){"use strict";n.d(t,"a",(function(){return u})),n.d(t,"b",(function(){return a})),n.d(t,"c",(function(){return l})),n.d(t,"d",(function(){return c})),n.d(t,"e",(function(){return o})),n.d(t,"f",(function(){return b})),n.d(t,"g",(function(){return s})),n.d(t,"h",(function(){return i})),n.d(t,"i",(function(){return r})),n.d(t,"j",(function(){return d})),n.d(t,"k",(function(){return p})),n.d(t,"l",(function(){return g})),n.d(t,"m",(function(){return m}));const r="shown",i="hidden",o="beforeClose",s="focusContent",u="ArrowDown",a="ArrowUp",l="End",c="Enter",d="Home",m="Space",p="absolute",g="fixed",b="gl-new-dropdown-contents"},XiQx:function(e,t,n){"use strict";n.d(t,"a",(function(){return c}));var r=n("0M2I"),i=n("V5u/"),o=n("Qog8"),s=n("7z1+"),u=n("BglX"),a=n("Pyw5"),l=n.n(a);const c="gl-new-dropdown-item";const d={name:u.c,ITEM_CLASS:c,components:{BLink:r.a},props:{item:{type:Object,required:!1,default:null,validator:s.c}},computed:{isLink(){var e,t;return"string"==typeof(null===(e=this.item)||void 0===e?void 0:e.href)||"string"==typeof(null===(t=this.item)||void 0===t?void 0:t.to)},isCustomContent(){return Boolean(this.$scopedSlots.default)},itemComponent(){const{item:e}=this;return this.isLink?{is:r.a,attrs:{href:e.href,to:e.to,...e.extraAttrs},listeners:{click:this.action}}:{is:"button",attrs:{...null==e?void 0:e.extraAttrs,type:"button"},listeners:{click:()=>{var t;null==e||null===(t=e.action)||void 0===t||t.call(void 0,e),this.action()}}}},listIndex(){var e,t;return null!==(e=this.item)&&void 0!==e&&null!==(t=e.extraAttrs)&&void 0!==t&&t.disabled?null:0},componentIndex(){var e,t;return null!==(e=this.item)&&void 0!==e&&null!==(t=e.extraAttrs)&&void 0!==t&&t.disabled?null:-1},wrapperClass(){var e,t;return null!==(e=null===(t=this.item)||void 0===t?void 0:t.wrapperClass)&&void 0!==e?e:""},wrapperListeners(){const e={keydown:this.onKeydown};return this.isCustomContent&&(e.click=this.action),e}},methods:{onKeydown(e){const{code:t}=e;if(t===i.d||t===i.m)if(this.isCustomContent)this.action();else{Object(o.k)(e);const t=new MouseEvent("click",{bubbles:!0,cancelable:!0});var n;if(this.isLink)this.$refs.item.$el.dispatchEvent(t);else null===(n=this.$refs.item)||void 0===n||n.dispatchEvent(t)}},action(){this.$emit("action",this.item)}}};const m=l()({render:function(){var e=this,t=e.$createElement,n=e._self._c||t;return n("li",e._g({class:[e.$options.ITEM_CLASS,e.wrapperClass],attrs:{tabindex:e.listIndex,"data-testid":"disclosure-dropdown-item"}},e.wrapperListeners),[e._t("default",(function(){return[n(e.itemComponent.is,e._g(e._b({ref:"item",tag:"component",staticClass:"gl-new-dropdown-item-content",attrs:{tabindex:e.componentIndex}},"component",e.itemComponent.attrs,!1),e.itemComponent.listeners),[n("span",{staticClass:"gl-new-dropdown-item-text-wrapper"},[e._t("list-item",(function(){return[e._v("\n          "+e._s(e.item.text)+"\n        ")]}))],2)])]}))],2)},staticRenderFns:[]},void 0,d,void 0,!1,void 0,!1,void 0,void 0,void 0);t.b=m},i7S8:function(e,t,n){"use strict";n.d(t,"w",(function(){return o})),n.d(t,"B",(function(){return s})),n.d(t,"L",(function(){return u})),n.d(t,"N",(function(){return a})),n.d(t,"p",(function(){return l})),n.d(t,"C",(function(){return c})),n.d(t,"j",(function(){return d})),n.d(t,"J",(function(){return m})),n.d(t,"K",(function(){return p})),n.d(t,"F",(function(){return g})),n.d(t,"E",(function(){return b})),n.d(t,"I",(function(){return f})),n.d(t,"H",(function(){return v})),n.d(t,"G",(function(){return _})),n.d(t,"s",(function(){return h})),n.d(t,"m",(function(){return M})),n.d(t,"n",(function(){return j})),n.d(t,"b",(function(){return q})),n.d(t,"a",(function(){return H})),n.d(t,"D",(function(){return R})),n.d(t,"o",(function(){return Y})),n.d(t,"A",(function(){return P})),n.d(t,"z",(function(){return U})),n.d(t,"q",(function(){return F})),n.d(t,"r",(function(){return K})),n.d(t,"f",(function(){return V})),n.d(t,"l",(function(){return W})),n.d(t,"t",(function(){return Q})),n.d(t,"k",(function(){return ee})),n.d(t,"v",(function(){return te})),n.d(t,"u",(function(){return ne})),n.d(t,"M",(function(){return re})),n.d(t,"i",(function(){return ie})),n.d(t,"y",(function(){return oe})),n.d(t,"h",(function(){return se})),n.d(t,"x",(function(){return ue})),n.d(t,"g",(function(){return ae})),n.d(t,"c",(function(){return le})),n.d(t,"d",(function(){return ce})),n.d(t,"e",(function(){return de}));var r=n("/lV4"),i=n("qLpH");const o="project-select",s=200,u="gl-bg-green-100",a="gl-bg-orange-100",l="gl-bg-red-100",c="members_invited_successfully",d={ALL:"all",DESCENDANT_GROUPS:"descendant_groups"},m="all",p="saml_provider_id",g="button",b="invite_members",f="text-emoji",v="dropdown-text-emoji",_="dropdown-text",h="invite_members_modal",M="invite_project_members_modal",j="project-members-page",w=Object(r.s__)("InviteMembersModal|Invite members"),y=Object(r.s__)("InviteMembersModal|GitLab is better with colleagues!"),I=Object(r.s__)("InviteMembersModal|How about inviting a colleague or two to join you?"),x=Object(r.s__)("InviteMembersModal|You're inviting members to the %{strongStart}%{name}%{strongEnd} group."),O=Object(r.s__)("InviteMembersModal|You're inviting members to the %{strongStart}%{name}%{strongEnd} project."),k=Object(r.s__)("InviteMembersModal|Congratulations on creating your project, you're almost there!"),S=Object(r.s__)("InviteMembersModal|Username, name or email address"),E=Object(r.s__)("InviteMembersModal|Select members or type email addresses"),A=Object(r.s__)("InviteMembersModal|Invite a group"),T=Object(r.s__)("InviteMembersModal|You're inviting a group to the %{strongStart}%{name}%{strongEnd} group."),C=Object(r.s__)("InviteMembersModal|You're inviting a group to the %{strongStart}%{name}%{strongEnd} project."),L=Object(r.s__)("InviteMembersModal|Inviting a group %{linkStart}adds its members to your group%{linkEnd}, including members who join after the invite. This might put your group over the free %{count} user limit."),$=Object(i.a)("user/project/members/sharing_projects_groups",{anchor:"invite-a-group-to-a-group"}),D=Object(r.s__)("InviteMembersModal|Inviting a group %{linkStart}adds its members to your project%{linkEnd}, including members who join after the invite. This might put your group over the free %{count} user limit."),B=Object(i.a)("user/project/members/sharing_projects_groups",{anchor:"invite-a-group-to-a-project"}),G=Object(r.s__)("InviteMembersModal|Select a group to invite"),z=Object(r.s__)("InviteMembersModal|Search for a group to invite"),q=Object(r.s__)("InviteMembersModal|Select a role"),H=Object(r.s__)("InviteMembersModal|Access expiration date (optional)"),R=Object(r.s__)("InviteMembersModal|Members were successfully added"),Y=Object(r.s__)("InviteMembersModal|Something went wrong"),P=Object(r.s__)("InviteMembersModal|%{linkStart}Read more%{linkEnd} about role permissions"),U=Object(r.s__)("InviteMembersModal|%{linkStart}Read more%{linkEnd} about access expiration"),F=Object(r.s__)("InviteMembersModal|Invite"),K=Object(r.s__)("InviteMembersModal|Manage members"),V=Object(r.s__)("InviteMembersModal|Cancel"),W=Object(r.s__)("InviteMembersModal|Close invite team members"),X=Object(r.s__)("InviteMembersModal|Review the invite errors and try again:"),N=Object(r.s__)("InviteMembersModal|Show more (%{count})"),Z=Object(r.s__)("InviteMembersModal|Show less"),J=Object(r.s__)("InviteMembersModal|Please add members to invite"),Q={modal:{default:{title:w},celebrate:{title:y,intro:I}},toGroup:{default:{introText:x}},toProject:{default:{introText:O},celebrate:{introText:k}},searchField:S,placeHolder:E,toastMessageSuccessful:R,memberErrorListText:X,collapsedErrors:N,expandedErrors:Z,emptyInvitesAlertText:J},ee={title:A,toGroup:{introText:T,notificationText:L,notificationLink:$},toProject:{introText:C,notificationText:D,notificationLink:B},searchField:G,placeHolder:z,toastMessageSuccessful:R},te="over_limit_modal_viewed",ne="invite_celebration_modal",re=Object(r.s__)("InviteMembersModal|You only have space for %{count} more %{members} in %{name}"),ie=Object(r.s__)("InviteMembersModal|You've reached your %{count} %{members} limit for %{name}"),oe="reached",se="close",ue=Object(r.s__)("InviteMembersModal|To invite new users to this top-level group, you must remove existing users. You can still add existing users from the top-level group, including any subgroups and projects.").concat(Object(r.s__)("InviteMembersModal| To get more members, the owner of this top-level group can %{trialLinkStart}start a trial%{trialLinkEnd} or %{upgradeLinkStart}upgrade%{upgradeLinkEnd} to a paid tier.")),ae=Object(r.s__)("InviteMembersModal|To get more members an owner of the group can %{trialLinkStart}start a trial%{trialLinkEnd} or %{upgradeLinkStart}upgrade%{upgradeLinkEnd} to a paid tier."),le=Object(r.s__)("InviteMembersModal|You must purchase more seats for your subscription before this amount of users can be added."),ce=Object(r.s__)("InviteMembersModal|Purchase more seats"),de="seat_limit_exceeded_error"}}]);
//# sourceMappingURL=initInviteMembersTrigger.f5e61ca7.chunk.js.map