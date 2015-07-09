#ifndef _RAZER_DAEMON_H_
#define _RAZER_DAEMON_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <signal.h>
#include <dlfcn.h>


#include "../lib/razer_chroma.h"

#ifdef USE_DBUS
	#include <dbus/dbus.h>
#endif


struct razer_effect;
struct razer_daemon;
struct razer_fx_render_node;

typedef int (*razer_effect_open)(struct razer_fx_render_node *render);//called when effect is added to a render node
typedef int (*razer_effect_close)(struct razer_fx_render_node *render);//called when effect is removed from a render node
typedef int (*razer_effect_reset)(struct razer_fx_render_node *render);//called every time a effect is called for the first time in a render node(loops trigger too)
typedef int (*razer_effect_update)(struct razer_fx_render_node *render);//called every frame - returns 0 if execution is completed
typedef int (*razer_effect_key_event)(struct razer_fx_render_node *render);//handler for key events
typedef int (*razer_effect_dbus_event)(struct razer_fx_render_node *render);//handler for dbus events

typedef void (*razer_effect_init)(struct razer_daemon *daemon);//called when libray is loaded
typedef void (*razer_effect_shutdown)(struct razer_daemon *daemon);//called when libray is unloaded


#define RAZER_PARAMETER_TYPE_STRING 1
#define RAZER_PARAMETER_TYPE_INT 2
#define RAZER_PARAMETER_TYPE_FLOAT 3
#define RAZER_PARAMETER_TYPE_RGB 4
#define RAZER_PARAMETER_TYPE_RENDER_NODE 5
//#define RAZER_PARAMETER_TYPE_F 5
//#define RAZER_PARAMETER_TYPE_MATH_OP 5
//TODO RANDOM TYPES (Range included)


#define RAZER_COMPOSE_MODE_MIX 1


struct razer_parameter
{
	int id;
	char *key;
	char *description;
	//void *value;
	unsigned long value;
	int type;
};

struct razer_parameters
{
	int num;
	int items_uid;
	struct razer_parameter **items;
};

struct razer_effect
{
	int id;
	char *name;
	char *description;
	int fps;
	struct razer_parameters *parameters;
	int input_usage_mask;//bitmask showing the effects usage of inputs
	razer_effect_open open;
	razer_effect_close close;
	razer_effect_update update;
	razer_effect_key_event key_event;
	razer_effect_dbus_event dbus_event;
};

struct razer_fx_render_node 
{
	int id;
	char *description;
	int compose_mode;
	struct razer_rgb_frame *input_frame;//TODO add second/multiple input frame(s) here for mixing ?
	struct razer_rgb_frame *second_input_frame;//TODO add second/multiple input frame(s) here for mixing ?
	struct razer_rgb_frame *output_frame;
	int input_frame_linked_uid;//-1==empty input frame buffer 
	int second_input_frame_linked_uid;//-1==empty input frame buffer 
	int output_frame_linked_uid;//-1==internal frame buffer used ; 0 == daemon output buffer is used
	struct razer_daemon *daemon;
	struct razer_effect *effect;//create a copy of the effect struct for every render node
	float opacity;
	float second_opacity;
	struct razer_fx_render_node *parent;
	int subs_num;
	struct razer_fx_render_node **subs; //TODO reuse subs for computational only effects (seeding /randomizing values etc)
	unsigned long start_ms;
	int limit_render_time_ms;
	int continue_chain;
	int loop_count;// -1 == no looping
	struct razer_fx_render_node *next;
};

struct razer_daemon 
{
	struct razer_chroma *chroma;
	int running;
	int fps;
	struct razer_fx_render_node *render_node;
	struct razer_fx_render_node **render_nodes;
	int render_nodes_num;
	int is_render_nodes_dirty;
	struct razer_fx_render_node *return_render_node;
	struct razer_rgb_frame *frame_buffer;
	int frame_buffer_linked_uid;//need to know if i have to free it or it is a link to another buffer
	//struct razer_keys *keys;
	//struct razer_keys_locks locks;
	int libs_num;
	void **libs;
	int effects_uid;
	int effects_num;
	struct razer_effect **effects;
	//int effect_instances_num;
	//struct razer_effect **effect_instances;
	int fx_render_nodes_uid;
	int fx_render_nodes_num;
	struct razer_fx_render_node **fx_render_nodes;
	#ifdef USE_DBUS
		DBusConnection *dbus;
	#endif
};

struct razer_queue
{
	void **items;
	int num;
	int pos;
};


void *daemon_load_fx_lib(struct razer_daemon *daemon,char *fx_pathname);

struct razer_effect *daemon_create_effect(void);
void daemon_free_effect(struct razer_effect **effect);
int daemon_register_effect(struct razer_daemon *daemon,struct razer_effect *effect);
int daemon_unregister_effect(struct razer_daemon *daemon,struct razer_effect *effect);
struct razer_effect *daemon_get_effect(struct razer_daemon *daemon,int uid);

struct razer_fx_render_node *daemon_create_render_node(struct razer_daemon *daemon,struct razer_effect *effect,int input_render_node_uid,int second_input_render_node_uid,int output_render_node_uid,char *description);
int daemon_register_render_node(struct razer_daemon *daemon,struct razer_fx_render_node *render_node);
struct razer_rgb_frame *daemon_create_rgb_frame(void);
void daemon_free_rgb_frame(struct razer_rgb_frame **frame);
struct razer_fx_render_node *daemon_get_render_node(struct razer_daemon *daemon,int uid);


struct razer_parameter *daemon_create_parameter_string(char *key,char *description,char *value);
struct razer_parameter *daemon_create_parameter_float(char *key,char *description,float value);
struct razer_parameter *daemon_create_parameter_int(char *key,char *description,int value);
struct razer_parameter *daemon_create_parameter_rgb(char *key,char *description,struct razer_rgb *value);
struct razer_parameter *daemon_create_parameter_render_node(char *key,char *description,struct razer_fx_render_node *value);

void daemon_set_parameter_string(struct razer_parameter *parameter,char *value);
void daemon_set_parameter_float(struct razer_parameter *parameter,float value);
void daemon_set_parameter_int(struct razer_parameter *parameter,int value);
void daemon_set_parameter_rgb(struct razer_parameter *parameter,struct razer_rgb *value);
void daemon_set_parameter_render_node(struct razer_parameter *parameter,struct razer_fx_render_node *value);

struct razer_queue *daemon_create_queue(void);
void daemon_free_queue(struct razer_queue **queue);

struct razer_parameter *daemon_create_parameter(void);
struct razer_parameter *daemon_copy_parameter(struct razer_parameter *parameter);
void daemon_free_parameter(struct razer_parameter **parameter);
void daemon_free_parameters(struct razer_parameters **parameters);


int daemon_add_parameter(struct razer_parameters *parameters,struct razer_parameter *parameter);
struct razer_parameter *daemon_remove_parameter(struct razer_parameters *parameters,char *key,int type);
struct razer_parameter *daemon_get_parameter(struct razer_parameters *parameters,char *key,int type);
struct razer_parameter *daemon_get_parameter_by_index(struct razer_parameters *parameters,int index);
char *daemon_get_parameter_string(struct razer_parameter *parameter);
float daemon_get_parameter_float(struct razer_parameter *parameter);
int daemon_get_parameter_int(struct razer_parameter *parameter);
struct razer_rgb *daemon_get_parameter_rgb(struct razer_parameter *parameter);
struct razer_fx_render_node *daemon_get_parameter_render_node(struct razer_parameter *parameter);

void daemon_render_node_add_sub(struct razer_fx_render_node *render_node,struct razer_fx_render_node *sub_node);


#endif