# CHANGELOG



## v0.1.0 (2023-11-03)

### Feature

* feat(ci): add semantic-release workflow (#62) ([`2067eb4`](https://github.com/citros-garden/drone/commit/2067eb42c2da1033667f7eb4bc5979e209a8e44e))

* feat(px4-config): adding ekf2 parameters (#59) ([`3bc4d1c`](https://github.com/citros-garden/drone/commit/3bc4d1cc1a96ff5c937a48403ce61b09e907139d))

### Fix

* fix(citros_notebooks): update (#60) ([`e0e4079`](https://github.com/citros-garden/drone/commit/e0e4079f757f2370e99bad075dd8353fd48f4ec1))

* fix(production Dockerfile): copy from workspace, override QOS profile (#52)

* fix(qos-profile): automatically changes qos profile to reliable

* fix(production Dockerfile): copy from workspace, override QOS profile ([`ad935af`](https://github.com/citros-garden/drone/commit/ad935afe35b9f6f5c645186439866915dcaa51b0))

* fix(qos-profile): chaning from best_effort to reliable (#51) ([`79dc12d`](https://github.com/citros-garden/drone/commit/79dc12d92a20196afbabee5be22ebd467c0ce1b3))

### Unknown

* doc(citros_template): add notebooks (#58) ([`80ffd24`](https://github.com/citros-garden/drone/commit/80ffd244a68ce53ee38810a9b8bed4a8968771a5))

* doc(readme): update (#57)

* doc(readme): update

* fix(Dockerfile): cleanup ([`e0d9bb8`](https://github.com/citros-garden/drone/commit/e0d9bb88136c16a3e495bf5ca124f157dc6a22a3))

* Doc/update readme (#50)

* doc(readme): updates the main readme

* doc(readme): update ([`10d25b2`](https://github.com/citros-garden/drone/commit/10d25b22ecacb1a9422ec94681acd686136a6704))

* Feat/remove unused xrce topics (#49)

* adding method for replacing dds_topic.yaml

* creatring /tmp/px4 if not exist ([`801e211`](https://github.com/citros-garden/drone/commit/801e2117f747c3c71c77e57dcf9170d818d1600c))

* using citros version 1.2.25 ([`d8b9199`](https://github.com/citros-garden/drone/commit/d8b91997c1c520c9cc48490872ca975c0ab1c911))

* adding mcap storage to the production dockerfile ([`60f6b80`](https://github.com/citros-garden/drone/commit/60f6b80f87a32b91aa4d177b76502e1294c135d2))

* Fix/production dockerfile (#47)

* working version of the production dockerfile

* adding docker-in-docker feature

* removing recorder from launch file ([`190c15f`](https://github.com/citros-garden/drone/commit/190c15f9b87c45d148c5e4ac0603d1ae6c1601f1))

* remove namespace (#45) ([`03a938b`](https://github.com/citros-garden/drone/commit/03a938b6abac7b2f9897cee29e5600b8f68ba350))

* updating sdf_modifier docs ([`ca8c675`](https://github.com/citros-garden/drone/commit/ca8c675d0d76352276a8088a9855e0414430535b))

* Feature/adding citros sim run dir (#43)

* using CITROS_SIM_RUN_DIR environment variable

* running locally works

* working from citros and from terminal ([`4a2fc99`](https://github.com/citros-garden/drone/commit/4a2fc99624299e21046ac66d5a8bf7f69a8f15d4))

* installing params.yaml of empty packages (#42) ([`af9680c`](https://github.com/citros-garden/drone/commit/af9680c8beb744c3a97adc556bc663cb74d723df))

* removing the extra : (#40) ([`fab17a7`](https://github.com/citros-garden/drone/commit/fab17a73eccf826a913e8c61ae6c12ee2b37c7b5))

* using package for non-ros parameters (#38) ([`355098b`](https://github.com/citros-garden/drone/commit/355098b19838277ea45ee5dc86c6f00c2b0f8cda))

* Bug/fix px4 parameter handler (#37)

* adding new modifier

* working configuration of the new px4 parameters handler

* adding docs

* using px4_modifier in the launch file ([`06d85e9`](https://github.com/citros-garden/drone/commit/06d85e921278c5a7c9b96c9485b6f5cf94f619a8))

* Feature/remove empty packages (#36)

* fix logging issue - using logger instead of print

* remove empty packages ([`8fafa70`](https://github.com/citros-garden/drone/commit/8fafa702e169354fd6d487ef6b30d59bd71f3ba5))

* Feature/code cleanup (#35)

* clean init function

* rename position setpoint message

* matching between target_msg and current_setpoint

* rename basic functions

* minor code cleanup ([`6dac0b3`](https://github.com/citros-garden/drone/commit/6dac0b3fd9eacbd5615a492e389fc563c8ec9716))

* Feature/removing time sleep (#34)

* sending arm command if vehicle not armed

* working configuration without sleep

* code cleanup

* more code cleanup

* fixing utils function bug ([`0a50308`](https://github.com/citros-garden/drone/commit/0a50308e04e71d50df0c17bb6cc787e90fa60845))

* setting debug level to info (#33) ([`4a8554b`](https://github.com/citros-garden/drone/commit/4a8554b6575a3cec25afc29328a6857494a5310a))

* sourcing overlay in the install.sh ([`ff7cbb2`](https://github.com/citros-garden/drone/commit/ff7cbb26527f8eca56bf9b9b1476e28fd25cb09d))

* Improving parsers (#30)

* some cleanup in the ros workspace

* commint for citros init

* rename px4_offboard package

* commit for citros

* using xmltodict for parsing parameters

* using xmltodict package to handle parameters

* more code cleanup

* adding docs to the parameterconventor

* renaming sdf modifier

* logging with logger instead of print

* some code cleanup

* handle lists in the dict

* parameters path as dicts

* creating yaml_to_sdf file

* trying to deal with list in the end-parameter

* handling list in last element

* code cleanup

* more code cleanup

* code cleanup

* add config in json format

* more code cleanup

* improving docs

* catch exception for invalid json file

* code cleanup

* unable to solve the px4 parameters

* removing px4 from submodules

* adding PX4 to the gitignore

* fixing building errors

* setting config.json with true paths

* using windy world in the sitl

* fixing sdf file paths, using jinja instead of sdf

* minor cleanup

* improving docs

* adding i/o defenitions ([`62a9b01`](https://github.com/citros-garden/drone/commit/62a9b01007db8a594de306b001feac03a4b160c7))

* default bag saving in mcap (#28)

* chaning qos profiles

* using build_depend in px4_msgs

* some minor cleanup

* working configuration with mcap bags

* saving bags in mcap ([`af06871`](https://github.com/citros-garden/drone/commit/af068710e374f6efb70a014718f6fde8b6973ce2))

* adding simulation setup (#21) ([`cd25063`](https://github.com/citros-garden/drone/commit/cd25063e99c77605301b27509297e62faffd843e))

* Adding wind (#20)

* adding world parameters

* working world parameters

* changing to default wind parameters ([`fbd0d4e`](https://github.com/citros-garden/drone/commit/fbd0d4ee1aa62302b92c0ddb7348a7749a4a2182))

* Add sensors config (#19)

* adding sensors parameters

* fixing px4 parameter printings ([`b00b60d`](https://github.com/citros-garden/drone/commit/b00b60d54403511c2b1aa64ca4f946caf881e06f))

* setting default as headless (#18) ([`54129ad`](https://github.com/citros-garden/drone/commit/54129addcc87c868fb94938face8b625423d3638))

* working vizualisation scenario (#17) ([`bba9726`](https://github.com/citros-garden/drone/commit/bba9726e3d143034a5cb955eafc5a8a4fe23ca70))

* Update issue templates (#16) ([`93137ff`](https://github.com/citros-garden/drone/commit/93137ff1fc9af0966adafcca01d2042f026f417e))

* adding citros commands to the readme (#15) ([`82822c8`](https://github.com/citros-garden/drone/commit/82822c8f1732705fb5c16dcd1be7879f8456ed89))

* installing citros in the deployment docker (#14) ([`cfb0523`](https://github.com/citros-garden/drone/commit/cfb0523311c6a6e984264c1329af68f6f9080a33))

* update docker commands with drone instead of px4 (#13) ([`86cf34a`](https://github.com/citros-garden/drone/commit/86cf34a3fac909c1fb8793f5f54ac3f40d72cfad))

* shutdown simulation when offboard done (#12) ([`51ea1a8`](https://github.com/citros-garden/drone/commit/51ea1a8373fd471f445657f1304a3511e6de3eb2))

* creating /tmp/px4 folder during installation ([`ba4b2df`](https://github.com/citros-garden/drone/commit/ba4b2df1518a01031e759d50e5f1a505793332d4))

* fixing path in px4 setup (#11) ([`05e0347`](https://github.com/citros-garden/drone/commit/05e0347d141a2c194c909f5d096cf23e328f4a10))

* fixing deps in the project (#10) ([`7216357`](https://github.com/citros-garden/drone/commit/721635700784a5b28f6a6ba55cbd4bcd3731583e))

* Update README.md ([`c6d8951`](https://github.com/citros-garden/drone/commit/c6d8951882838554a910650bc3d52a2e7cbfc940))

* working with v1.14 of px4 (#9) ([`9b4b165`](https://github.com/citros-garden/drone/commit/9b4b1659e3115195c68d504ae306cf03d894103f))

* Update README.md (#8) ([`32d3fab`](https://github.com/citros-garden/drone/commit/32d3fab4b668ce0f83da9f768eb868b6e59d64bd))

* Deployment dockerfile (#7)

* adding deployment dockerfile

* almost working version of deployment docker

* working version of the deployment docker ([`6973bda`](https://github.com/citros-garden/drone/commit/6973bda3a42e195ded32ed1ab8bd1f56e84bf414))

* Parsing px4 parameters (#6)

* working configuration of px4 parameters

* working px4 parameters ([`7b79d40`](https://github.com/citros-garden/drone/commit/7b79d403dc2555fcf29ff4405be64644fd7f2fae))

* Parsing body parameters (#5)

* adding parameters script

* body parameters works ([`02d7e41`](https://github.com/citros-garden/drone/commit/02d7e410bcb85a4626fe6206a439909dd7659774))

* offboard logic (#4)

* fixing local position subscription bug

* adding logic

* publishing current state

* working configuration of the offboard

* adding parameters

* more code cleanups ([`df1706f`](https://github.com/citros-garden/drone/commit/df1706fd0a242e40bade9282e19362b64e43414d))

* adding vscode tasks (#3) ([`1bedf3d`](https://github.com/citros-garden/drone/commit/1bedf3d23355a7671d97701c1d1d7b9941c367d0))

* cleanup ([`eaa952d`](https://github.com/citros-garden/drone/commit/eaa952db57ad9667ae2b8d53aee360e299d135e4))

* adding offboard mode guard ([`5cbbbea`](https://github.com/citros-garden/drone/commit/5cbbbeac2f5992c212f84f94f1b3a4c129a2b37d))

* working configuration of launc file ([`0381b2d`](https://github.com/citros-garden/drone/commit/0381b2d5fa1ab7c3d062c822edbe2d40bb536980))

* fixing missing examples when compiling ([`846df91`](https://github.com/citros-garden/drone/commit/846df913954ba82d4ce6d1caa2276bf79e88a8c6))

* removing examples ([`46340e3`](https://github.com/citros-garden/drone/commit/46340e3a39bf30740bb14105ff748a6c3305246f))

* trying to fix submodules probelm ([`3220641`](https://github.com/citros-garden/drone/commit/3220641352e6f98101d3881d4ebe7626cccd46c3))

* adding rosbridge at launch ([`de5fa8c`](https://github.com/citros-garden/drone/commit/de5fa8c92f226770ac817c5677451058c3e49a88))

* working offb + dds agent from launch file ([`7f7a4fd`](https://github.com/citros-garden/drone/commit/7f7a4fdd44f103d5b5b5c9613f96856455c0f87d))

* update readme ([`2f50d78`](https://github.com/citros-garden/drone/commit/2f50d78398aeda64a04703065c97df12b2410d8e))

* cleanup (#2) ([`06fc239`](https://github.com/citros-garden/drone/commit/06fc239a38fda4bd477e7e240d0d3e9411b542d1))

* fixing gazebo installation problem, removing build files from git ([`c54443e`](https://github.com/citros-garden/drone/commit/c54443ef1693f09f34d2840e62608763fcd4b5b1))

* setting up the dev env (#1)

* setting up the dev env

* adding gazebo installtion ([`82fd962`](https://github.com/citros-garden/drone/commit/82fd962b797ebc15c0d0debf94446fcf652e6d5d))

* template cleanout ([`36183af`](https://github.com/citros-garden/drone/commit/36183afda6a508f9c74f46bc7b4436ac884e39c2))

* Initial commit ([`e7ea4e2`](https://github.com/citros-garden/drone/commit/e7ea4e2edf988c0b398481e3b41977c2f84461ab))
