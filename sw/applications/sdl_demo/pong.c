#include "FreeRTOS.h"
#include "task.h"
#include <SDL.h>
#include "paranut_logo.h"
#include "freertos_logo.h"
#include "ees_logo.h"
#include <stdlib.h>
#include "vears.h"


// SETTINGS ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Video settings
#define video_width             640
#define video_height            480
// Size of ParaNut logo used to determine bounding box
#define ParanutLogo_width       64
#define ParanutLogo_height      64
// Size of FreeRTOS logo used to determine bounding box
#define FreertosLogo_width      64
#define FreertosLogo_height     64
// Size of EES logo used to determine bounding box
#define EESLogo_width           64
#define EESLogo_height          64
// Milliseconds since 1970
#define rand_seed               1607976829
// Distance from border of screen
#define space_video_left_right  25
// Maximum and minimum speed
#define x_speed_min             5
#define x_speed_max             11
#define y_speed_min             5
#define y_speed_max             11
// Parameters for PD controller
#define P       1
#define a_max   10
#define v_max   30
#define dt      0.9f

// TYPEDEFS //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Defines the structure of a racket
typedef struct {
    // Acceleration
    float a;
    // Velocity
    float v;
    // Y position
    float y;
    // Associated SDL_Rect structure that will be updated
    // by a call to update_racket_y
    SDL_Rect *rect;
} tRacket;

// VARIABLES //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Bounding boxes to determine size and position of the surface to be drawn
SDL_Rect paranut_rect;      // rectangles for the bmp file rendering
SDL_Rect ees_rect;
// The ball rectangle
SDL_Rect freertos_rect;
// The left racket (ParaNut logo)
tRacket paranut_racket = {};
// The right racket (EES logo)
tRacket ees_racket = {};
// Velocity vector for FreeRTOS logo movement (aka ball)
int freertos_direction[2];
// Background colors
int red = 0xFF; 
int green = 0x00;
int blue = 0xFF;
// Set when ball left field, reset when new round is started
int out_of_field = 0; 

int x_min; 
int x_max; 

int y_min;
int y_max;

int x_map;         
int y_map;

SDL_Surface *screen, *paranut, *ees, *freertos;
SDL_Renderer *renderer;

// FUNCTIONS //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int abs(int val);

void change_color_rand(void);

void update_racket_y(float random_factor, tRacket *racket);

void reset(void);

void update_rackets(void);

void render_task(void *pvParamaters) {
    int selected_screen = 0;
    SDL_Rect paranut_rect_delay_1 = paranut_rect, paranut_rect_delay_2 = paranut_rect;
    SDL_Rect ees_rect_delay_1 = ees_rect, ees_rect_delay_2 = ees_rect;
    SDL_Rect freertos_rect_delay_1 = freertos_rect, freertos_rect_delay_2 = freertos_rect;

    SDL_Surface *screen_1 = screen;
    SDL_Surface *screen_2 = SDL_ConvertSurface(screen, screen_1->format, 0);

    for(;;) {
        // Things are not allowed to be moved during rendering (currently only freertos logo would even move)
        // Prevents scheduler from entering game_task
        taskENTER_CRITICAL();
        /* Clear old area first */
        if (selected_screen) {
            SDL_FillRect(screen, &paranut_rect_delay_1, SDL_MapRGB(screen->format,red,green,blue));
            SDL_FillRect(screen, &ees_rect_delay_1, SDL_MapRGB(screen->format,red,green,blue));
            SDL_FillRect(screen, &freertos_rect_delay_1, SDL_MapRGB(screen->format,red,green,blue));
        } else {
            SDL_FillRect(screen, &paranut_rect_delay_2, SDL_MapRGB(screen->format,red,green,blue));
            SDL_FillRect(screen, &ees_rect_delay_2, SDL_MapRGB(screen->format,red,green,blue));
            SDL_FillRect(screen, &freertos_rect_delay_2, SDL_MapRGB(screen->format,red,green,blue));
        }
        
        // check if ball has left the playground
        if (out_of_field) {
            // Reset rackets and ball
            reset();
            // Randomize background color
            change_color_rand();

            // Clear both front and back buffer with new color 
            // (optimally safe which one, and do it when safe, but that's too much to be bothered with, just to demonstrate FreeRTOS)
            SDL_FillRect(screen_1, NULL, SDL_MapRGB(screen->format,red,green,blue));
            SDL_FillRect(screen_2, NULL, SDL_MapRGB(screen->format,red,green,blue));
        }
        else {
            update_rackets();
        }
        
        // Save which area will need to be cleaned depending on selected screen
        if (selected_screen) {
            paranut_rect_delay_1 = paranut_rect;
            ees_rect_delay_1 = ees_rect;
            freertos_rect_delay_1 = freertos_rect;
        } else {
            paranut_rect_delay_2 = paranut_rect;
            ees_rect_delay_2 = ees_rect;
            freertos_rect_delay_2 = freertos_rect;
        }
        // RENDER EVERYTHING ////////////////////////////////////////////////////////////
        /* Clear the rendering surface with the specified color */
        SDL_SetRenderDrawColor(renderer, red, green, blue, 0xFF);   
        // Draw bmp files 
        SDL_BlitSurface(paranut, 0, screen, &paranut_rect);
        SDL_BlitSurface(ees, 0, screen, &ees_rect);
        SDL_BlitSurface(freertos, 0, screen, &freertos_rect);

        // Set the new framebuffer location to be shown by VEARS
        vears_image_show (VEARS_BASEADDR, screen->pixels);
        if (selected_screen) {
            selected_screen = 0;
            screen = screen_1;
        } else {
            selected_screen = 1;
            screen = screen_2;
        }
        taskEXIT_CRITICAL();
        // Give up time for other task
        taskYIELD();
    }

}

void game_task(void *pvParamaters) {
    int collision_left_right = 0;
    int last_collision_left_right = 0;
    for(;;) {
        // Reset as soon as game_task can handle it
        // Also put in a delay here, in case cleaning should be too fast
        // (which it currently isn't)
        if (out_of_field) {
            out_of_field = 0;
        }
        

        y_map = freertos_rect.y;
        x_map = freertos_rect.x;
        
        unsigned int map_pos = 0;

        x_map += freertos_direction[0];
        if (x_map > x_max) {
            x_map = x_min + (x_map - x_max);
            map_pos |= 0x01UL;
            collision_left_right = 1;
        }
        else if (x_map < x_min) {
            x_map = x_max - (x_min - x_map);
            map_pos |= 0x01UL;
            collision_left_right = 1;
        }

        y_map += freertos_direction[1];
        if (y_map > y_max) {
            y_map = y_min + (y_map - y_max);
            map_pos |= 0x02UL;
        }
        else if (y_map < y_min) {
            y_map = y_max - (y_min - y_map);
            map_pos |= 0x02UL;
        }

        // flip vector components 
        if (map_pos & 0x01UL) {
            freertos_direction[0] = -freertos_direction[0];
        }  
        if (map_pos & 0x02UL) {
            freertos_direction[1] = -freertos_direction[1];
        }
     
        int collision = 0;
        if (last_collision_left_right != collision_left_right) {
            // collision left or right happened!
            collision = 1;
        }
       
        last_collision_left_right = collision_left_right;
        collision_left_right = 0;
        
        // Project x and y position back onto coordinate system
        switch (map_pos) {

        case 0:
            freertos_rect.x = x_map;
            freertos_rect.y = y_map;
            break;
        case 1:
            freertos_rect.x = x_min + x_max - x_map;    // flip x
            freertos_rect.y = y_map;
            break;
        case 2:
            freertos_rect.x = x_map;
            freertos_rect.y = y_max - y_map;            // flip y
            break;
        case 3:
            freertos_rect.x  = x_min + x_max - x_map;   // flip x and y
            freertos_rect.y  = y_max - y_map; 
            break;
        }


        // left or right collision with racket => add racket y-speed/2 to ball y-speed, factor 2 is just choosen, not physically correct! 
        if (collision) {
            if ((freertos_rect.x - paranut_rect.x) < (ees_rect.x - freertos_rect.x)) {
                if (((paranut_rect.y + paranut_rect.h) >= (freertos_rect.y - abs(freertos_direction[1]))) &&
                    (paranut_rect.y <= (freertos_rect.y + freertos_rect.h + abs(freertos_direction[1])))) {
                    // left collision
                    freertos_direction[1] += ((int)paranut_racket.v / 2);   
                }
                else {
                    out_of_field = 1;
                }   
            }
            else {
                if (((ees_rect.y + ees_rect.h) >= (freertos_rect.y - abs(freertos_direction[1]))) &&
                    (ees_rect.y <= (freertos_rect.y + freertos_rect.h + abs(freertos_direction[1])))) {
                    // right collision
                    freertos_direction[1] += ((int)ees_racket.v / 2); 
                }
                else {
                    out_of_field = 1;
                }        
            }
        }

        // Limit Y velocity of ball
        if (freertos_direction[1] > y_speed_max) {
            freertos_direction[1] = y_speed_max;  
        }
        else if(freertos_direction[1] < -y_speed_max) {
            freertos_direction[1] = -y_speed_max;
        }
        // Give up time for other task
        taskYIELD();
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int pong() {    
    // Inform SDL about our desired video driver
    setenv("SDL_VIDEODRIVER", "dummy", 1);
    //SDL_VideoInit("dummy");
    // Use a constant time value as random seed (obviously not truly random)
    srand(rand_seed);

    // Create an SDL Window with specified dimensions
    SDL_Window *window = SDL_CreateWindow("FreeRTOS SDL Demo",SDL_WINDOWPOS_UNDEFINED,SDL_WINDOWPOS_UNDEFINED,video_width,video_height, 0);
    // Obtain underlying SDL surface
    screen = SDL_GetWindowSurface(window);
    // Print framebuffer location (to be passed to VEARS)
    printf("Framebuffer located at: %p\n", screen->pixels);

    // Read BMP files from memory
    // This is done by passing the pointer to the file contents as well as the length of the file
    paranut = SDL_LoadBMP_RW(SDL_RWFromMem((void*)bin2c_paranut_bmp, sizeof(bin2c_paranut_bmp)), 0);
    ees = SDL_LoadBMP_RW(SDL_RWFromMem((void*)bin2c_ees_bmp, sizeof(bin2c_ees_bmp)), 0);
    freertos = SDL_LoadBMP_RW(SDL_RWFromMem((void*)bin2c_freertos_bmp, sizeof(bin2c_freertos_bmp)), 0);

    // Create a software renderer used for clearing the screen
    renderer = SDL_CreateSoftwareRenderer(screen);
    if(!renderer) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Render creation for surface fail : %s\n",SDL_GetError());
        return 1;
    }
    
    // Sets magenta as transparency (color key) for both ParaNut and EES logo
    if (SDL_SetColorKey(paranut, SDL_TRUE, SDL_MapRGB(paranut->format,255,0,255))) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "paranut bmp creation for colorkey fail : %s\n",SDL_GetError());
    }
    
    if (SDL_SetColorKey(ees, SDL_TRUE, SDL_MapRGB(ees->format,255,0,255))) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "ees bmp creation for colorkey fail : %s\n",SDL_GetError());
    }

    // Initialize SDL rectangles/bounding boxes used for rendering
    // paranut is the left ping pong racket
    paranut_rect.h = ParanutLogo_height;
    paranut_rect.w = ParanutLogo_width;

    // ees is the right ping pong racket
    ees_rect.h = EESLogo_height;
    ees_rect.w = EESLogo_width;

    // freertos
    freertos_rect.h = FreertosLogo_height;
    freertos_rect.w = FreertosLogo_width;
    
    paranut_racket.rect = &paranut_rect;
    ees_racket.rect = &ees_rect;

    paranut_racket.y = paranut_rect.y;
    ees_racket.y = ees_rect.y;

    // Set racket and ball positions and initial velocity 
    reset();
    // Point VEARS to framebuffer for presentation
    vears_init (VEARS_BASEADDR, screen->pixels);

    // FreeRTOS task creation
    TaskHandle_t xHandle;
    BaseType_t xReturned;
    // Create a render task and a game task
    // Will be used by scheduler to switch between each other once they're done or their time is over
    xReturned = xTaskCreate(render_task, "RenderTask", 128*2, (void *)1, tskIDLE_PRIORITY + 1, &xHandle);
    if (xReturned == pdPASS) {
        puts("Successfully created RenderTask\n");
    }

    xReturned = xTaskCreate(game_task, "GameTask", 128*2, (void *)1, tskIDLE_PRIORITY + 1, &xHandle);
    if (xReturned == pdPASS) {
        puts("Successfully created GameTask\n");
    }

    return 0;
}


//---------------------------------------------------------------------------------------------
/** @brief Update Y positions for both rackets (left and right) */
void update_rackets(void) {
    
    float p = ((freertos_rect.x - paranut_rect.w - space_video_left_right) / 
                (video_width - (2.0 * space_video_left_right) - paranut_rect.w - ees_rect.w - freertos_rect.w));
        
    update_racket_y (p, &paranut_racket);
    update_racket_y ((1.0f - p), &ees_racket);
}

//---------------------------------------------------------------------------------------------
void update_racket_y (float random_factor, tRacket *racket) {
        float y_offset_rand = random_factor * ((((rand() / (float)RAND_MAX) * 2) - 1) * (video_height - racket->rect->h));
        float y_soll = (freertos_rect.y + y_offset_rand) * (1 - random_factor) + ((video_height / 2) - (racket->rect->h / 2)) * random_factor; 

        // Calculate acceleration
        racket->a = (y_soll - racket->y) * P; 
        if (racket->a >= a_max) {
            racket->a = a_max;
        }
        else if (racket->a <= -a_max) {
            racket->a = -a_max;
        }

        // Apply acceleration to velocity
        racket->v = (racket->v + (dt * racket->a));  
        if (racket->v >= v_max) {
            racket->v = v_max;
        }
        else if (racket->v <= -v_max) {
            racket->v = -v_max;
        }

        // Apply velocity to position
        racket->y = (racket->y + (dt * racket->v));  
        if (racket->y <= 0.0f) {
            racket->y = 0.0f;
        }
        else if ((racket->y + racket->rect->h) >= video_height) {
            racket->y = (video_height - racket->rect->h);
        }
        // Update SDL rectangle (render position)
        racket->rect->y = (int)racket->y;
}

//---------------------------------------------------------------------------------------------
/** @brief Reset racket and ball positions and initial velocity. */
void reset(void) {
    // Reset rendering positions of rackets and balls
    // paranut is the left ping pong racket
    paranut_rect.x = space_video_left_right;      
    paranut_rect.y = ((video_height / 2) - (ParanutLogo_height / 2));

    // ees is the right ping pong racket
    ees_rect.x = (video_width - EESLogo_width - space_video_left_right);      
    ees_rect.y = (video_height / 2) - (ParanutLogo_height / 2);

    // freertos
    freertos_rect.x = ((video_width / 2) - (FreertosLogo_width / 2));      // random start position at right window half
    freertos_rect.y = ((video_height / 2) - (FreertosLogo_height / 2));

    paranut_racket.y = paranut_rect.y;
    ees_racket.y = ees_rect.y;

    // direction vector x,y start initializing from 1...min_speed
    int sign;
    if (rand() % 2) {
        sign = 1;
    }
    else {
        sign = -1;
    }

    freertos_direction[0] = (1 + abs(x_speed_min) + (rand() % (1 + abs(x_speed_max - x_speed_min)))) * sign; 
    
    if (rand() % 2) {
        sign = 1;
    }
    else {
        sign = -1;
    }

    freertos_direction[1] = (1 + abs(y_speed_min) + (rand() % (1 + abs(y_speed_max - y_speed_min)))) * sign; 


    x_min = paranut_rect.x + paranut_rect.w;
    x_max = ees_rect.x - freertos_rect.w;
    x_map = freertos_rect.x;
    
    y_min = 0;
    y_max = video_height - freertos_rect.h;
    y_map = freertos_rect.y;
}

//---------------------------------------------------------------------------------------------
/** @brief Randomize background color */
void change_color_rand(void) {
    red = (rand() % 0xFF);
    green = (rand() % 0xFF);
    blue = (rand() % 0xFF);
}
